// hello.rs
//
// simple test of semihosting
//
// > openocd -f openocd.cfg
// > cargo run --example hello

#![no_main]
#![no_std]

use panic_semihosting as _;

#[rtic::app(device = atsamx7x_hal::target_device, dispatchers = [])]
mod app {
    use atsamx7x_hal as hal;
    use cortex_m_semihosting::hprintln;
    use hal::ehal::watchdog::WatchdogDisable;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        hprintln!("init");
        // Disable the watchdog.
        hal::watchdog::Watchdog::new(ctx.device.WDT).disable();

        (Shared {}, Local {}, init::Monotonics())
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle");

        loop {}
    }
}
