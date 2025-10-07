#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use alloc::format;

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use esp_hal::Config;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Flex, Input, InputConfig, Level, Output, OutputConfig, Pin, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;

use log::{error, info, warn};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_net::Ipv4Address;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Timer};

use tosca::route::{LightOffRoute, LightOnRoute, Route};

use tosca_esp32c3::devices::light::Light;
use tosca_esp32c3::events::broker::BrokerData;
use tosca_esp32c3::events::interrupt::Notifier;
use tosca_esp32c3::events::{EventsConfig, EventsManager};
use tosca_esp32c3::mdns::Mdns;
use tosca_esp32c3::net::NetworkStack;
use tosca_esp32c3::response::{ErrorResponse, OkResponse, SerialResponse};
use tosca_esp32c3::server::Server;
use tosca_esp32c3::state::{State, ValueFromRef};
use tosca_esp32c3::wifi::Wifi;

use tosca_drivers::am312::Am312;
use tosca_drivers::bh1750::{Address, Bh1750, Resolution};
use tosca_drivers::dht22::{Dht22, Measurement};
use tosca_drivers::ds18b20::{Ds18b20, Ds18b20Error};

use embedded_graphics::mono_font::ascii::FONT_9X15;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::text::renderer::TextRenderer;
use embedded_graphics::text::{Baseline, Text};

use ssd1306::mode::BufferedGraphicsModeAsync;
use ssd1306::{I2CDisplayInterface, Ssd1306Async, prelude::*};

const MAX_HEAP_SIZE: usize = 128 * 1024;
const MILLISECONDS_TO_WAIT: u64 = 100;

// Socket buffer size.
const TX_SIZE: usize = 2048;
// Server buffer size.
const RX_SIZE: usize = 4096;
// Maximum number of allowed headers in a request.
const MAXIMUM_HEADERS_COUNT: usize = 32;
// Timeout.
const TIMEOUT: u32 = 15 * 1000;

// Signal that indicates a change in the LIGHT's state.
static NOTIFY_LIGHT: Signal<CriticalSectionRawMutex, LightInput> = Signal::new();
// Signal that indicates a change in the LIGHT's mode.
static NOTIFY_MODE: Signal<CriticalSectionRawMutex, LightMode> = Signal::new();
// Signal that indicates a change in the TEMPERATURE value.
static NOTIFY_TEMPERATURE: Signal<CriticalSectionRawMutex, f32> = Signal::new();
// Atomic signal to enable and disable the toggle task.
static TOGGLE_CONTROLLER: AtomicBool = AtomicBool::new(false);
// Atomic signal to control light mode.
static LIGHT_MODE: AtomicU8 = AtomicU8::new(LightMode::Manual as u8);

// Calibration time of the AM312 motion sensor.
const AM312_CALIBRATION_TIME_S: u64 = 15;
// Threshold of the BH1750 Ambient light sensor.
const BH1750_THRESHOLD_LX: f32 = 3.0;

// Shared I²C bus wrapped in a static Mutex to allow safe concurrent access
// by multiple I²C devices (BH1750, SSD1306) across async tasks.
static I2C_BUS: static_cell::StaticCell<
    Mutex<CriticalSectionRawMutex, I2c<'static, esp_hal::Async>>,
> = static_cell::StaticCell::new();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[toml_cfg::toml_config]
struct DeviceConfig {
    #[default("")]
    ssid: &'static str,
    #[default("")]
    password: &'static str,
    #[default("")]
    broker_address: &'static str,
    #[default(0)]
    broker_port: u16,
}

#[derive(Clone, Copy)]
enum LightInput {
    On,
    Off,
    Toggle,
    Switch,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum LightMode {
    Manual,
    MotionDetection,
    AmbientLight,
}

#[embassy_executor::task]
async fn toggle_switch(mut switch: Input<'static>) {
    let mut last_level = switch.level();

    loop {
        // Wait until the switch changes state.
        switch.wait_for_any_edge().await;

        // Simple debounce: wait 50ms and check the state again.
        Timer::after_millis(MILLISECONDS_TO_WAIT).await;
        let current_level = switch.level();

        // Only proceed if the state actually changed.
        if current_level == last_level {
            continue;
        }
        last_level = current_level;
        info!("Switch toggled!");

        // Only react if the light mode is Manual.
        if LIGHT_MODE.load(Ordering::Relaxed) != LightMode::Manual as u8 {
            info!("Switch toggled, but light mode is not manual, ignoring.");
            continue;
        }

        // Disable automatic toggle task and signal the light task.
        TOGGLE_CONTROLLER.store(false, Ordering::Relaxed);
        NOTIFY_LIGHT.signal(LightInput::Switch);
    }
}

// Turn the light on.
#[inline]
fn light_on(light: &mut Output<'static>) {
    light.set_high();
    info!("Light is on!");
}

// Turn the light off.
#[inline]
fn light_off(light: &mut Output<'static>) {
    light.set_low();
    info!("Light is off!");
}

// Toggle the light.
#[inline]
fn toggle_light(light: &mut Output<'static>) {
    // Toggle the LIGHT on or off based on its current state.
    if light.is_set_high() {
        light_off(light);
    } else {
        light_on(light);
    }
}

async fn change_light(light: AnyPin<'static>, notifier: Notifier<bool>) {
    // Output light.
    let mut light = Output::new(light, Level::Low, OutputConfig::default());

    // Notify the external process of the light status when the task is initiated.
    notifier.update_event(light.is_set_high()).await;

    loop {
        // Wait until a signal is received before proceeding.
        let light_input = NOTIFY_LIGHT.wait().await;

        match light_input {
            LightInput::On => {
                light_on(&mut light);
            }
            LightInput::Off => {
                light_off(&mut light);
            }
            LightInput::Switch => {
                toggle_light(&mut light);
            }
            LightInput::Toggle => {
                while TOGGLE_CONTROLLER.load(Ordering::Relaxed) {
                    toggle_light(&mut light);
                    notifier.update_event(light.is_set_high()).await;
                    // Pause for 5 seconds before toggling the light again.
                    Timer::after_secs(5).await;
                }
            }
        }
        // Notify at the end of each input operation.
        notifier.update_event(light.is_set_high()).await;

        // Wait for a specified duration before restarting the loop.
        Timer::after_millis(MILLISECONDS_TO_WAIT).await;
    }
}

async fn change_mode(notifier: Notifier<u8>) {
    // Notify the external process of the LIGHT mode when the task is initiated
    notifier
        .update_event(LIGHT_MODE.load(Ordering::Relaxed))
        .await;

    loop {
        // Wait until a signal is received before proceeding.
        let mode = NOTIFY_MODE.wait().await as u8;

        // Update the atomic value.
        LIGHT_MODE.store(mode, Ordering::Relaxed);

        // Notify at the end of each input operation.
        notifier.update_event(mode).await;

        // Wait for a specified duration before restarting the loop.
        Timer::after_millis(MILLISECONDS_TO_WAIT).await;
    }
}

async fn change_temperature(notifier: Notifier<f32>) {
    loop {
        // Wait until a signal is received before proceeding.
        let temperature = NOTIFY_TEMPERATURE.wait().await;

        // Notify at the end of each input operation.
        notifier.update_event(temperature).await;

        // Wait for a specified duration before restarting the loop.
        Timer::after_millis(MILLISECONDS_TO_WAIT).await;
    }
}

#[inline]
async fn signal_light(light_input: LightInput) {
    // Disable the toggle task.
    TOGGLE_CONTROLLER.store(false, Ordering::Relaxed);

    // Wait for a specified amount of time before notifying the LIGHT.
    Timer::after_millis(MILLISECONDS_TO_WAIT).await;

    // Notify light to change its current state.
    NOTIFY_LIGHT.signal(light_input);
}

#[inline]
async fn notify_light(
    light_input: LightInput,
    message: &str,
    text_message: &'static str,
) -> Result<SerialResponse, ErrorResponse> {
    signal_light(light_input).await;

    info!("{message}");

    // Returns a serial response from a text.
    Ok(SerialResponse::text(text_message))
}

async fn turn_light_on() -> Result<SerialResponse, ErrorResponse> {
    notify_light(
        LightInput::On,
        "Light turned on through PUT route!",
        "Light on",
    )
    .await
}

async fn turn_light_off() -> Result<SerialResponse, ErrorResponse> {
    notify_light(
        LightInput::Off,
        "Light turned off through PUT route!",
        "Light off",
    )
    .await
}

struct RequestCounter(&'static AtomicU32);

impl ValueFromRef for RequestCounter {
    fn value_from_ref(&self) -> Self {
        Self(self.0)
    }
}

#[allow(clippy::unnecessary_wraps)]
fn stateful_toggle(
    State(RequestCounter(request_counter)): State<RequestCounter>,
) -> Result<OkResponse, ErrorResponse> {
    // Obtain the current request counter value.
    let old_value = request_counter.load(Ordering::Relaxed);
    // Increment the request counter value.
    request_counter.store(old_value + 1, Ordering::Relaxed);

    info!("Request number: {request_counter:?}");

    // Enable the toggle task.
    TOGGLE_CONTROLLER.store(true, Ordering::Relaxed);

    // Notify light.
    NOTIFY_LIGHT.signal(LightInput::Toggle);

    info!("Light togglight through GET route!");

    Ok(OkResponse::new())
}

#[allow(clippy::unnecessary_wraps)]
fn set_manual_mode() -> Result<OkResponse, ErrorResponse> {
    NOTIFY_MODE.signal(LightMode::Manual);

    info!("Setting manual mode");

    Ok(OkResponse::new())
}

#[allow(clippy::unnecessary_wraps)]
fn set_motion_detection_mode() -> Result<OkResponse, ErrorResponse> {
    NOTIFY_MODE.signal(LightMode::MotionDetection);

    info!("Setting motion detection mode");

    Ok(OkResponse::new())
}

#[allow(clippy::unnecessary_wraps)]
fn set_ambient_light_mode() -> Result<OkResponse, ErrorResponse> {
    NOTIFY_MODE.signal(LightMode::AmbientLight);

    info!("Setting ambient light mode");

    Ok(OkResponse::new())
}

#[embassy_executor::task]
pub async fn handle_motion_sensor(mut am312: Am312<Input<'static>, Delay>) {
    // Wait for calibration time.
    info!("Waiting for motion sensor calibration...");
    Timer::after_secs(AM312_CALIBRATION_TIME_S).await;
    info!("Motion sensor calibration ended");

    loop {
        // Skip if not in MotionDetection mode.
        if LIGHT_MODE.load(Ordering::Relaxed) != LightMode::MotionDetection as u8 {
            Timer::after_secs(2).await;
            continue;
        }

        // Detect motion.
        am312
            .wait_for_motion_start()
            .await
            .expect("Error while waiting for motion detection");
        info!("Motion detected");

        // Turn on the light only if we are still in MotionDetection mode.
        if LIGHT_MODE.load(Ordering::Relaxed) == LightMode::MotionDetection as u8 {
            signal_light(LightInput::On).await;
        }

        // Detect motion end.
        am312
            .wait_for_motion_end()
            .await
            .expect("Error while waiting for motion detection");
        info!("Motion ended");

        // Turn off the light only if we are still in MotionDetection mode.
        if LIGHT_MODE.load(Ordering::Relaxed) == LightMode::MotionDetection as u8 {
            signal_light(LightInput::Off).await;
        }
    }
}

#[embassy_executor::task]
async fn handle_ambient_light_sensor(
    mut bh1750: Bh1750<
        I2cDevice<
            'static,
            CriticalSectionRawMutex,
            esp_hal::i2c::master::I2c<'static, esp_hal::Async>,
        >,
        Delay,
    >,
) {
    // Power on.
    bh1750.power_on().await.expect("Error while powering on");

    // Start continous measurement with HIGH resolution.
    bh1750
        .start_continuous_measurement(Resolution::High)
        .await
        .expect("Error while starting continous measurement");

    loop {
        // Check if on ambient light mode.
        if LIGHT_MODE.load(Ordering::Relaxed) != LightMode::AmbientLight as u8 {
            Timer::after_secs(2).await;
            continue;
        }

        // Read a single continous measurement.
        let lux = bh1750
            .read_continuous_measurement()
            .await
            .expect("Error while reading continuous measurement");

        info!("Luminosity level: {lux:.2} lx");

        if lux < BH1750_THRESHOLD_LX {
            signal_light(LightInput::On).await;
        } else {
            signal_light(LightInput::Off).await;
        }

        Timer::after_secs(2).await;
    }
}

#[embassy_executor::task]
async fn handle_temperature_humidity_sensor(mut dht22: Dht22<Flex<'static>, Delay>) {
    loop {
        match dht22.read() {
            Ok(Measurement {
                temperature,
                humidity,
            }) => {
                info!("Temperature: {temperature:.1} °C, Humidity: {humidity:.1} %");
            }
            Err(e) => {
                error!("DHT22 read error: {e:?}");
            }
        }

        Timer::after_secs(10).await;
    }
}

type Display128x64 = Ssd1306Async<
    I2CInterface<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, esp_hal::Async>>>,
    DisplaySize128x64,
    BufferedGraphicsModeAsync<DisplaySize128x64>,
>;

fn draw_temperature_display<D>(
    display: &mut D,
    style: &MonoTextStyle<'_, BinaryColor>,
    display_size: Size,
    temperature: f32,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    // Helper conversion.
    fn u32_to_i32(u: u32) -> i32 {
        i32::try_from(u).expect("u32 too large for i32")
    }

    let temp_label = "Temp: ";
    let temp_value = format!("{temperature:.1}");

    // Measure text length.
    let prefix_metrics = style.measure_string(temp_label, Point::zero(), Baseline::Top);
    let temp_metrics = style.measure_string(&temp_value, Point::zero(), Baseline::Top);

    let white_space = 2;
    let total_width = u32_to_i32(prefix_metrics.bounding_box.size.width)
        + u32_to_i32(temp_metrics.bounding_box.size.width)
        + white_space
        + 6
        + u32_to_i32(style.font.character_size.width);

    let x_start = (u32_to_i32(display_size.width) - total_width) / 2;
    let y_start =
        (u32_to_i32(display_size.height) - u32_to_i32(temp_metrics.bounding_box.size.height)) / 2;

    // Write "Temp: " label.
    Text::with_baseline(
        temp_label,
        Point::new(x_start, y_start),
        *style,
        Baseline::Top,
    )
    .draw(display)?;

    // Write numeric value.
    let x_number = x_start + u32_to_i32(prefix_metrics.bounding_box.size.width);
    Text::with_baseline(
        &temp_value,
        Point::new(x_number, y_start),
        *style,
        Baseline::Top,
    )
    .draw(display)?;

    // Draw ° circle.
    let degree_pos = Point::new(
        x_number + u32_to_i32(temp_metrics.bounding_box.size.width) + white_space,
        y_start,
    );
    Circle::new(degree_pos, 4)
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display)?;

    // Draw C.
    let x_c = x_number + u32_to_i32(temp_metrics.bounding_box.size.width) + white_space + 6;
    Text::with_baseline("C", Point::new(x_c, y_start), *style, Baseline::Top).draw(display)?;

    Ok(())
}

#[embassy_executor::task]
async fn handle_ds18b20_temperature(
    mut ds18b20: Ds18b20<Flex<'static>, Delay>,
    mut display: Display128x64,
) {
    // Display text style.
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15)
        .text_color(BinaryColor::On)
        .build();

    // SSD1306 128x64.
    let display_size = Size::new(128, 64);

    loop {
        match ds18b20.read_temperature() {
            // Discard any 85 °C reading. This can occur after a power-on reset or
            // due to voltage glitches. For this demo scenario, temperatures
            // never reach 85 °C, so it's safe to treat it as invalid.
            Ok(85.0) => {
                Timer::after_millis(100).await;
                continue;
            }
            Ok(temp) => {
                info!("DS18B20 temperature: {temp:.1} °C");

                // Notify a value change in order to trigger an event.
                NOTIFY_TEMPERATURE.signal(temp);

                display.clear_buffer();

                if let Err(e) = draw_temperature_display(&mut display, &style, display_size, temp) {
                    error!("Display draw error: {e:?}");
                }

                display
                    .flush()
                    .await
                    .expect("Error while flushing the display");
            }
            Err(Ds18b20Error::CrcMismatch) => {
                warn!("DS18B20 CRC mismatch, invalid reading ignored");
            }
            Err(e) => {
                error!("DS18B20 read error: {e:?}");
            }
        }

        Timer::after_secs(30).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: MAX_HEAP_SIZE);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    info!("ESP RTOS started!");

    let rng = Rng::new();

    // Retrieve device configuration.
    let device_config = DEVICE_CONFIG;

    let interfaces = Wifi::configure(peripherals.WIFI, spawner)
        .expect("Failed to configure Wi-Fi")
        .connect(device_config.ssid, device_config.password)
        .await
        .expect("Failed to connect to Wi-Fi");

    info!("Starting network stack...");

    let stack = NetworkStack::build::<12>(rng, interfaces.sta, spawner)
        .await
        .expect("Failed to create the network stack.");

    // Input switch.
    let switch = Input::new(
        peripherals.GPIO3,
        InputConfig::default().with_pull(Pull::Up),
    );

    // Input motion sensor.
    let motion_sensor = Input::new(peripherals.GPIO2, InputConfig::default());
    let delay = Delay {};
    let am312 = Am312::new(motion_sensor, delay);

    // I²C config.
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .expect("Error while configuring I²C")
        .with_sda(peripherals.GPIO8)
        .with_scl(peripherals.GPIO9)
        .into_async();

    let i2c_bus = Mutex::new(i2c);
    let i2c_bus = I2C_BUS.init(i2c_bus);

    // Ambient light sensor.
    let i2c_bh1750 = I2cDevice::new(i2c_bus);
    let bh1750 = Bh1750::new(i2c_bh1750, Delay {}, Address::Low);

    // Temperature sensor.
    let ds18b20_pin = Output::new(
        peripherals.GPIO10,
        Level::High,
        OutputConfig::default().with_drive_mode(esp_hal::gpio::DriveMode::OpenDrain),
    )
    .into_flex();
    ds18b20_pin.peripheral_input();
    let delay = Delay {};
    let ds18b20 = Ds18b20::new(ds18b20_pin, delay);

    // Display.
    let i2c_ssd1306 = I2cDevice::new(i2c_bus);
    let interface = I2CDisplayInterface::new(i2c_ssd1306);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    // Initialize the display.
    display
        .init()
        .await
        .expect("Error while initializing the display");
    display.clear_buffer();
    display
        .flush()
        .await
        .expect("Error while flushing the display");

    spawner
        .spawn(toggle_switch(switch))
        .expect("Impossible to spawn the task to toggle the switch");
    spawner
        .spawn(handle_motion_sensor(am312))
        .expect("Impossible to spawn the task to handle the AM312 motion sensor");
    spawner
        .spawn(handle_ambient_light_sensor(bh1750))
        .expect("Impossible to spawn the task to handle the BH1750 ambient light sensor");
    spawner
        .spawn(handle_ds18b20_temperature(ds18b20, display))
        .expect("Impossible to spawn the task to handle the DS18B20 temperature sensor");

    let request_counter = RequestCounter(mk_static!(AtomicU32, AtomicU32::new(0)));

    let device = Light::with_state(&interfaces.ap, request_counter)
        .turn_light_on_stateless_serial(
            LightOnRoute::put("On").description("Turn light on."),
            |_| async move { turn_light_on().await },
        )
        .turn_light_off_stateless_serial(
            LightOffRoute::put("Off").description("Turn light off."),
            |_| async move { turn_light_off().await },
        )
        .stateful_ok_route(
            Route::get("Toggle", "/toggle").description("Toggle."),
            |state, _| async move { stateful_toggle(state) },
        )
        .stateless_ok_route(
            Route::put("Manual mode", "/manual").description("Set the manual mode."),
            |_| async move { set_manual_mode() },
        )
        .stateless_ok_route(
            Route::put("Motion detection mode", "/motion-detection")
                .description("Set the motion detection mode."),
            |_| async move { set_motion_detection_mode() },
        )
        .stateless_ok_route(
            Route::put("Ambient light mode", "/ambient-light")
                .description("Set the ambient light mode."),
            |_| async move { set_ambient_light_mode() },
        )
        .build();

    let events_config = EventsConfig::new(
        spawner,
        stack,
        BrokerData::ip(
            device_config
                .broker_address
                .parse()
                .unwrap_or(Ipv4Address::LOCALHOST.into()),
            device_config.broker_port,
        ),
        device,
    );

    let device = EventsManager::config(events_config)
        .bool_event(
            "status",
            "Change the light status",
            change_light,
            peripherals.GPIO4.degrade(),
        )
        .u8_event_pinless("mode", "Change the light mode", change_mode)
        .f32_event_pinless(
            "temperature",
            "Change the temperature value",
            change_temperature,
        )
        .run_network_task()
        .await
        .expect("Failed to run the events manager");

    Server::<TX_SIZE, RX_SIZE, MAXIMUM_HEADERS_COUNT, _>::new(
        device,
        Mdns::new(rng)
            .service_type("_ascot")
            .hostname("ascot")
            .service("ascot"),
    )
    .keepalive_timeout(TIMEOUT)
    .run(stack, spawner)
    .await
    .expect("Failed to run a server");
}
