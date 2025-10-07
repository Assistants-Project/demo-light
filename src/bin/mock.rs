#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use esp_hal::Config;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pin, Pull};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;

use log::info;

use embassy_executor::Spawner;
use embassy_net::Ipv4Address;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;

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

// Signal that indicates a change in the LED's state.
static NOTIFY_LED: Signal<CriticalSectionRawMutex, LedInput> = Signal::new();
// Signal that indicates a change in the LED's mode.
static NOTIFY_MODE: Signal<CriticalSectionRawMutex, LedMode> = Signal::new();
// Atomic signal to enable and disable the toggle task.
static TOGGLE_CONTROLLER: AtomicBool = AtomicBool::new(false);
// Atomic signal to control led mode.
static LED_MODE: AtomicU8 = AtomicU8::new(LedMode::Manual as u8);

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
enum LedInput {
    On,
    Off,
    Toggle,
    Button,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum LedMode {
    Manual,
    MotionDetection,
    AmbientLight,
}

#[embassy_executor::task]
async fn press_button(mut button: Input<'static>) {
    loop {
        // Wait until the button is pressed.
        button.wait_for_rising_edge().await;
        info!("Button Pressed!");

        // Only react if the led mode is Manual.
        if LED_MODE.load(Ordering::Relaxed) != LedMode::Manual as u8 {
            info!("Button pressed, but led mode is not manual, ignoring.");
            continue;
        }

        // Disable the toggle task.
        TOGGLE_CONTROLLER.store(false, Ordering::Relaxed);

        // Wait for a specified amount of time before notifying the led.
        Timer::after_millis(MILLISECONDS_TO_WAIT).await;

        // Notify led to change its current state.
        NOTIFY_LED.signal(LedInput::Button);

        // Wait for some time before starting the loop again.
        Timer::after_millis(MILLISECONDS_TO_WAIT).await;
    }
}

// Turn the led on.
#[inline]
fn led_on(led: &mut Output<'static>) {
    led.set_low();
    info!("Led is on!");
}

// Turn the led off.
#[inline]
fn led_off(led: &mut Output<'static>) {
    led.set_high();
    info!("Led is off!");
}

// Toggle the led.
#[inline]
fn toggle_led(led: &mut Output<'static>) {
    // Toggle the LED on or off based on its current state.
    if led.is_set_high() {
        led_on(led);
    } else {
        led_off(led);
    }
}

async fn change_led(led: AnyPin<'static>, notifier: Notifier<bool>) {
    // Output led.
    let mut led = Output::new(led, Level::High, OutputConfig::default());

    // Notify the external process of the LED status when the task is initiated
    notifier.update_event(led.is_set_low()).await;

    loop {
        // Wait until a signal is received before proceeding.
        let led_input = NOTIFY_LED.wait().await;

        match led_input {
            LedInput::On => {
                led_on(&mut led);
            }
            LedInput::Off => {
                led_off(&mut led);
            }
            LedInput::Button => {
                toggle_led(&mut led);
            }
            LedInput::Toggle => {
                while TOGGLE_CONTROLLER.load(Ordering::Relaxed) {
                    toggle_led(&mut led);
                    notifier.update_event(led.is_set_low()).await;
                    // Pause for 5 seconds before toggling the led again.
                    Timer::after_secs(5).await;
                }
            }
        }
        // Notify at the end of each input operation.
        notifier.update_event(led.is_set_low()).await;

        // Wait for a specified duration before restarting the loop.
        Timer::after_millis(MILLISECONDS_TO_WAIT).await;
    }
}

async fn change_mode(notifier: Notifier<u8>) {
    // Notify the external process of the LED mode when the task is initiated
    notifier
        .update_event(LED_MODE.load(Ordering::Relaxed))
        .await;

    loop {
        // Wait until a signal is received before proceeding.
        let mode = NOTIFY_MODE.wait().await as u8;

        // Update the atomic value.
        LED_MODE.store(mode, Ordering::Relaxed);

        // Notify at the end of each input operation.
        notifier.update_event(mode).await;

        // Wait for a specified duration before restarting the loop.
        Timer::after_millis(MILLISECONDS_TO_WAIT).await;
    }
}

#[inline]
async fn signal_led(led_input: LedInput) {
    // Disable the toggle task.
    TOGGLE_CONTROLLER.store(false, Ordering::Relaxed);

    // Wait for a specified amount of time before notifying the LED.
    Timer::after_millis(MILLISECONDS_TO_WAIT).await;

    // Notify led to change its current state.
    NOTIFY_LED.signal(led_input);
}

#[inline]
async fn notify_led(
    led_input: LedInput,
    message: &str,
    text_message: &'static str,
) -> Result<SerialResponse, ErrorResponse> {
    signal_led(led_input).await;

    info!("{message}");

    // Returns a serial response from a text.
    Ok(SerialResponse::text(text_message))
}

async fn turn_light_on() -> Result<SerialResponse, ErrorResponse> {
    notify_led(LedInput::On, "Led turned on through PUT route!", "Light on").await
}

async fn turn_light_off() -> Result<SerialResponse, ErrorResponse> {
    notify_led(
        LedInput::Off,
        "Led turned off through PUT route!",
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

    // Notify led.
    NOTIFY_LED.signal(LedInput::Toggle);

    info!("Led toggled through GET route!");

    Ok(OkResponse::new())
}

#[allow(clippy::unnecessary_wraps)]
fn set_manual_mode() -> Result<OkResponse, ErrorResponse> {
    NOTIFY_MODE.signal(LedMode::Manual);

    info!("Setting manual mode");

    Ok(OkResponse::new())
}

#[allow(clippy::unnecessary_wraps)]
fn set_motion_detection_mode() -> Result<OkResponse, ErrorResponse> {
    NOTIFY_MODE.signal(LedMode::MotionDetection);

    info!("Setting motion detection mode");

    Ok(OkResponse::new())
}

#[allow(clippy::unnecessary_wraps)]
fn set_ambient_light_mode() -> Result<OkResponse, ErrorResponse> {
    NOTIFY_MODE.signal(LedMode::AmbientLight);

    info!("Setting ambient light mode");

    Ok(OkResponse::new())
}

#[embassy_executor::task]
pub async fn handle_motion_sensor() {
    loop {
        // Simulate a motion trigger every 10 seconds
        if LED_MODE.load(Ordering::Relaxed) == LedMode::MotionDetection as u8 {
            info!("Motion detected");
            signal_led(LedInput::On).await;

            Timer::after_secs(3).await;

            info!("Motion ended");
            signal_led(LedInput::Off).await;
        }

        Timer::after_secs(10).await;
    }
}

#[embassy_executor::task]
async fn handle_ambient_light_sensor() {
    let mut dark = false;

    loop {
        if LED_MODE.load(Ordering::Relaxed) == LedMode::AmbientLight as u8 {
            // Alternate between dark and light every 5s.
            dark = !dark;

            if dark {
                info!("Ambient light: DARK → turn light ON");
                signal_led(LedInput::On).await;
            } else {
                info!("Ambient light: BRIGHT → turn light OFF");
                signal_led(LedInput::Off).await;
            }
        }

        Timer::after_secs(5).await;
    }
}

async fn change_temperature(notifier: Notifier<f32>) {
    loop {
        // Generate a random temperature value between 18 and 27.
        let temp = 18 + (embassy_time::Instant::now().as_ticks() % 10);

        notifier.update_event(temp as f32).await;

        info!("Temperature updated: {temp}°C");

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

    // Input button.
    let button = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );

    spawner.spawn(press_button(button)).unwrap();
    spawner.spawn(handle_motion_sensor()).unwrap();
    spawner.spawn(handle_ambient_light_sensor()).unwrap();

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
            change_led,
            peripherals.GPIO8.degrade(),
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
