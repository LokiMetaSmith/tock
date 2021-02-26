//! Tock kernel for Opossum Pouch.
//!
//! It is based on nRF52833 SoC (Cortex M4 core with a BLE).

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]

use capsules::virtual_aes_ccm::MuxAES128CCM;
use capsules::virtual_alarm::VirtualMuxAlarm;
use kernel::common::dynamic_deferred_call::{DynamicDeferredCall, DynamicDeferredCallClientState};
use kernel::component::Component;
use kernel::hil::gpio::Configure;
use kernel::hil::gpio::Interrupt;
use kernel::hil::gpio::Output;
//use kernel::hil::i2c::I2CMaster;
use kernel::hil::led::LedHigh;
use kernel::hil::symmetric_encryption::AES128;
use kernel::hil::time::Counter;
use kernel::hil::usb::Client;
use kernel::mpu::MPU;
use kernel::Chip;
#[allow(unused_imports)]
use kernel::{capabilities, create_capability, debug, debug_gpio, debug_verbose, static_init};
use nrf52833::gpio::Pin;
use nrf52833::base_interrupts::PWM0;
use nrf52833::interrupt_service::Nrf52833DefaultPeripherals;
use nrf52_components::{self, UartChannel, UartPins};

// Kernel LED (same as microphone LED)
const LED_KERNEL_PIN: Pin = Pin::P0_24;
const LED_RED_PIN: Pin = Pin::P0_24;
const LED_GREEN_PIN: Pin = Pin::P0_25;
const LED_BLUE_PIN: Pin = Pin::P0_21;

const SWITCH: Pin = Pin::P0_11;

// power
const FLT: Pin = Pin::P0_13;
const PGOOD: Pin = Pin::P0_16;
const DEVSLP: Pin = Pin::P0_14;
const IMON: Pin = Pin::P0_02;
// GPIOs

// P0, P1 and P2 are used as ADC, comment them in the ADC section to use them as GPIO
//const _GPIO_P0: Pin = Pin::P0_02;
//const _GPIO_P1: Pin = Pin::P0_03;
//const _GPIO_P2: Pin = Pin::P0_04;
//const GPIO_P8: Pin = Pin::P0_10;
//const GPIO_P9: Pin = Pin::P0_09;
//const GPIO_P16: Pin = Pin::P1_02;

const UART_TX_PIN: Pin = Pin::P0_25;
const UART_RX_PIN: Pin = Pin::P0_21;

const SRC_MAC: u16 = 0xDEAD;
const PAN_ID:  u16 = 0xBEEF;

/// LED matrix
//const LED_MATRIX_COLS: [Pin; 5] = [Pin::P0_28, Pin::P0_11, Pin::P0_31, Pin::P1_05, Pin::P0_30];
//const LED_MATRIX_ROWS: [Pin; 5] = [Pin::P0_21, Pin::P0_22, Pin::P0_15, Pin::P0_24, Pin::P0_19];

// Speaker

//const SPEAKER_PIN: Pin = Pin::P0_00;

/// I2C pins for all of the sensors.
//const I2C_SDA_PIN: Pin = Pin::P0_16;
//const I2C_SCL_PIN: Pin = Pin::P0_08;

/// UART Writer for panic!()s.
pub mod io;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::procs::FaultResponse = kernel::procs::FaultResponse::Panic;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

static mut PROCESSES: [Option<&'static dyn kernel::procs::ProcessType>; NUM_PROCS] =
    [None; NUM_PROCS];

static mut CHIP: Option<&'static nrf52833::chip::NRF52<Nrf52833DefaultPeripherals>> = None;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];
// debug mode requires more stack space
// pub static mut STACK_MEMORY: [u8; 0x2000] = [0; 0x2000];

/// Supported drivers by the platform
pub struct Platform {
    ble_radio: &'static capsules::ble_advertising_driver::BLE<
        'static,
        nrf52833::ble_radio::Radio<'static>,
        VirtualMuxAlarm<'static, nrf52::rtc::Rtc<'static>>,
    >,
    pconsole: &'static capsules::process_console::ProcessConsole<
        'static,
        components::process_console::Capability,
    >,
    console: &'static capsules::console::Console<'static>,
    gpio: &'static capsules::gpio::GPIO<'static, nrf52833::gpio::GPIOPin<'static>>,
    led: &'static capsules::led::LedDriver<
        'static,
        LedHigh<'static, nrf52833::gpio::GPIOPin<'static>>,
    >,
    button: &'static capsules::button::Button<'static, nrf52::gpio::GPIOPin<'static>>,
    rng: &'static capsules::rng::RngDriver<'static>,
    //ninedof: &'static capsules::ninedof::NineDof<'static>,
    //lsm303agr: &'static capsules::lsm303agr::Lsm303agrI2C<'static>,
    temperature: &'static capsules::temperature::TemperatureSensor<'static>,
    ipc: kernel::ipc::IPC<NUM_PROCS>,
    adc: &'static capsules::adc::AdcVirtualized<'static>,
    analog_comparator: &'static capsules::analog_comparator::AnalogComparator<
        'static,
        nrf52833::acomp::Comparator<'static>,
    >,
    alarm: &'static capsules::alarm::AlarmDriver<
        'static,
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf52833::rtc::Rtc<'static>>,
    >,
//    buzzer: &'static capsules::buzzer_driver::Buzzer<
//        'static,
//        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf52833::rtc::Rtc<'static>>,
//    >,
    app_flash: &'static capsules::app_flash_driver::AppFlash<'static>,
}

impl kernel::Platform for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::Driver>) -> R,
    {
        match driver_num {
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            capsules::analog_comparator::DRIVER_NUM => f(Some(self.analog_comparator)),
            capsules::adc::DRIVER_NUM => f(Some(self.adc)),
            capsules::temperature::DRIVER_NUM => f(Some(self.temperature)),
            capsules::rng::DRIVER_NUM => f(Some(self.rng)),
            capsules::ble_advertising_driver::DRIVER_NUM => f(Some(self.ble_radio)),
            capsules::app_flash_driver::DRIVER_NUM => f(Some(self.app_flash)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}

/// Entry point in the vector table called on hard reset.
#[no_mangle]
pub unsafe fn reset_handler() {
    // Loads relocations and clears BSS
    nrf52833::init();

    let ppi = static_init!(nrf52833::ppi::Ppi, nrf52833::ppi::Ppi::new());
    // Initialize chip peripheral drivers
    let nrf52833_peripherals = static_init!(
        Nrf52833DefaultPeripherals,
        Nrf52833DefaultPeripherals::new(ppi)
    );

    // set up circular peripheral dependencies
    nrf52833_peripherals.init();
    let base_peripherals = &nrf52833_peripherals.nrf52;

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    //--------------------------------------------------------------------------
    // CAPABILITIES
    //--------------------------------------------------------------------------

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);
    let gpio_port = &nrf52833_peripherals.gpio_port;
    //--------------------------------------------------------------------------
    // DEBUG GPIO
    //--------------------------------------------------------------------------

    // Configure kernel debug GPIOs as early as possible. These are used by the
    // `debug_gpio!(0, toggle)` macro. We uconfigure these early so that the
    // macro is available during most of the setup code and kernel exection.
    kernel::debug::assign_gpios(
        Some(&nrf52833_peripherals.gpio_port[LED_KERNEL_PIN]),
        None,
        None,
    );

    //--------------------------------------------------------------------------
    // GPIO
    //--------------------------------------------------------------------------

    let gpio = components::gpio::GpioComponent::new(
        board_kernel,
        components::gpio_component_helper!(
            nrf52833::gpio::GPIOPin,
            // Used as ADC, comment them out in the ADC section to use them as GPIO
            // 0 => &nrf52833_peripherals.gpio_port[Pin::P0_0],
            // 1 => &nrf52833_peripherals.gpio_port[Pin::P0_1],
            // 2 => &nrf52833_peripherals.gpio_port[Pin::P0_2],
	        11 => &nrf52833_peripherals.gpio_port[Pin::P0_11],
            13 => &nrf52833_peripherals.gpio_port[Pin::P0_13],
            16 => &nrf52833_peripherals.gpio_port[Pin::P0_16],
            14 => &nrf52833_peripherals.gpio_port[Pin::P0_14],
            //8 => &nrf52833_peripherals.gpio_port[Pin::P0_8],
            //9 => &nrf52833_peripherals.gpio_port[Pin::P0_9],
            //16 => &nrf52833_peripherals.gpio_port[Pin::P0_16],
        ),
    )
    .finalize(components::gpio_component_buf!(nrf52833::gpio::GPIOPin));

    //--------------------------------------------------------------------------
    // Buttons
    //--------------------------------------------------------------------------
    let button = components::button::ButtonComponent::new(
        board_kernel,
        components::button_component_helper!(
            nrf52833::gpio::GPIOPin,
            (
                &nrf52833_peripherals.gpio_port[SWITCH],
                kernel::hil::gpio::ActivationMode::ActiveLow,
                kernel::hil::gpio::FloatingState::PullUp
            )
        ),
    )
    .finalize(components::button_component_buf!(nrf52833::gpio::GPIOPin));

    //--------------------------------------------------------------------------
    // Deferred Call (Dynamic) Setup
    //--------------------------------------------------------------------------

    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 3], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);

    //--------------------------------------------------------------------------
    // ALARM & TIMER
    //--------------------------------------------------------------------------

    let rtc = &base_peripherals.rtc;
    rtc.start();
    let mux_alarm = components::alarm::AlarmMuxComponent::new(rtc)
        .finalize(components::alarm_mux_component_helper!(nrf52::rtc::Rtc));
    let alarm = components::alarm::AlarmDriverComponent::new(board_kernel, mux_alarm)
        .finalize(components::alarm_component_helper!(nrf52::rtc::Rtc));

    //--------------------------------------------------------------------------
    // PWM & BUZZER
    //--------------------------------------------------------------------------

    use kernel::hil::time::Alarm;

    let mux_pwm = static_init!(
        capsules::virtual_pwm::MuxPwm<'static, nrf52::pwm::Pwm>,
        capsules::virtual_pwm::MuxPwm::new(&base_peripherals.pwm0)
    );



    //--------------------------------------------------------------------------
    // UART & CONSOLE & DEBUG
    //--------------------------------------------------------------------------

    base_peripherals.uarte0.initialize(
        nrf52::pinmux::Pinmux::new(UART_TX_PIN as u32),
        nrf52::pinmux::Pinmux::new(UART_RX_PIN as u32),
        None,
        None,
    );

    // Create a shared UART channel for the console and for kernel debug.
    let uart_mux = components::console::UartMuxComponent::new(
        &base_peripherals.uarte0,
        115200,
        dynamic_deferred_caller,
    )
    .finalize(());

    // Setup the console.
    let console = components::console::ConsoleComponent::new(board_kernel, uart_mux).finalize(());
    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new(uart_mux).finalize(());

    //--------------------------------------------------------------------------
    // RANDOM NUMBERS
    //--------------------------------------------------------------------------

    let rng = components::rng::RngComponent::new(board_kernel, &base_peripherals.trng).finalize(());

    //--------------------------------------------------------------------------
    // SENSORS
    //--------------------------------------------------------------------------
    let temperature = components::temperature::TemperatureComponent::new(board_kernel, &base_peripherals.temp).finalize(());
    //--------------------------------------------------------------------------
    // ADC
    //--------------------------------------------------------------------------
    base_peripherals.adc.calibrate();

    let adc_mux = components::adc::AdcMuxComponent::new(&base_peripherals.adc)
        .finalize(components::adc_mux_component_helper!(nrf52833::adc::Adc));

    // Comment out the following to use P0, P1 and P2 as GPIO
    let adc_syscall = components::adc::AdcVirtualComponent::new(board_kernel).finalize(
        components::adc_syscall_component_helper!(
            // ADC Ring 0 (P0)
            //components::adc::AdcComponent::new(
            //    &adc_mux,
            //    nrf52833::adc::AdcChannel::AnalogInput0
            //)
            //.finalize(components::adc_component_helper!(nrf52833::adc::Adc)),
            components::adc::AdcComponent::new(
                &adc_mux,
                nrf52833::adc::AdcChannelSetup::new(nrf52833::adc::AdcChannel::AnalogInput0)
            )
            .finalize(components::adc_component_helper!(nrf52833::adc::Adc)),
            // ADC Ring 1 (P1)
            components::adc::AdcComponent::new(
                &adc_mux,
                nrf52833::adc::AdcChannelSetup::new(nrf52833::adc::AdcChannel::AnalogInput1)
            )
            .finalize(components::adc_component_helper!(nrf52833::adc::Adc)),
            // ADC Ring 2 (P2)
            components::adc::AdcComponent::new(
                &adc_mux,
                nrf52833::adc::AdcChannelSetup::new(nrf52833::adc::AdcChannel::AnalogInput2)
            )
            .finalize(components::adc_component_helper!(nrf52833::adc::Adc))
        ),
    );
    let analog_comparator = components::analog_comparator::AcComponent::new(
        &base_peripherals.acomp,
        components::acomp_component_helper!(
            nrf52833::acomp::Channel,
            &nrf52833::acomp::CHANNEL_AC0
        ),
    )
    .finalize(components::acomp_component_buf!(
        nrf52833::acomp::Comparator
    ));
    //--------------------------------------------------------------------------
    // STORAGE
    //--------------------------------------------------------------------------

    // App Flash

    let app_flash =
        components::app_flash_driver::AppFlashComponent::new(board_kernel, &base_peripherals.nvmc)
            .finalize(components::app_flash_component_helper!(
                nrf52833::nvmc::Nvmc,
                512
            ));

    //--------------------------------------------------------------------------
    // WIRELESS
    //--------------------------------------------------------------------------

    let ble_radio =
        nrf52_components::BLEComponent::new(board_kernel, &base_peripherals.ble_radio, mux_alarm)
            .finalize(());

    //--------------------------------------------------------------------------
    // LEDs
    //--------------------------------------------------------------------------
    let led = components::led::LedsComponent::new(components::led_component_helper!(
        LedHigh<'static,nrf52833::gpio::GPIOPin>,
        LedHigh::new(&nrf52833_peripherals.gpio_port[LED_RED_PIN]),
        LedHigh::new(&nrf52833_peripherals.gpio_port[LED_GREEN_PIN]),
        LedHigh::new(&nrf52833_peripherals.gpio_port[LED_BLUE_PIN]),
    ))
    .finalize(components::led_component_buf!(
        LedHigh<'static,nrf52833::gpio::GPIOPin>
    ));

    //--------------------------------------------------------------------------
    // Process Console
    //--------------------------------------------------------------------------
    let pconsole =
        components::process_console::ProcessConsoleComponent::new(board_kernel, uart_mux)
            .finalize(());


    //--------------------------------------------------------------------------
    // FINAL SETUP AND BOARD BOOT
    //--------------------------------------------------------------------------

    // it seems that microbit v2 has no external clock
    nrf52_components::NrfClockComponent::new(&base_peripherals.clock).finalize(());;
    base_peripherals.clock.low_stop();
    base_peripherals.clock.high_stop();
    base_peripherals.clock.low_start();
    base_peripherals.clock.high_start();
    while !base_peripherals.clock.low_started() {}
    while !base_peripherals.clock.high_started() {}

    let platform = Platform {
        ble_radio: ble_radio,
        pconsole,
        console: console,
        button: button,
        led: led,
        gpio,
        temperature,
        analog_comparator,
        rng: rng,
        adc: adc_syscall,
        alarm: alarm,
        app_flash: app_flash,
        ipc: kernel::ipc::IPC::new(board_kernel, &memory_allocation_capability),
    };
    platform.pconsole.start();
    let chip = static_init!(
        nrf52833::chip::NRF52<Nrf52833DefaultPeripherals>,
        nrf52833::chip::NRF52::new(nrf52833_peripherals)
    );
    CHIP = Some(chip);
    //platform.process_console.start();
    debug!("Initialization complete. Entering main loop.");
    debug!("{}",&nrf52833::ficr::FICR_INSTANCE);
    //--------------------------------------------------------------------------
    // PROCESSES AND MAIN LOOP
    //--------------------------------------------------------------------------

    /// These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    kernel::procs::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            &_sapps as *const u8,
            &_eapps as *const u8 as usize - &_sapps as *const u8 as usize,
        ),
        core::slice::from_raw_parts_mut(
            &mut _sappmem as *mut u8,
            &_eappmem as *const u8 as usize - &_sappmem as *const u8 as usize,
        ),
        &mut PROCESSES,
        FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&PROCESSES)
        .finalize(components::rr_component_helper!(NUM_PROCS));
    board_kernel.kernel_loop(
        &platform,
        chip,
        Some(&platform.ipc),
        scheduler,
        &main_loop_capability,
    );
}
