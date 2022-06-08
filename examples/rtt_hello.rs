// rtt_hello.rs
//
// simple test of rtt
//
// > cargo embed --example hello

#![no_main]
#![no_std]

use panic_rtt_target as _;

#[rtic::app(device = atsamx7x_hal::target_device, dispatchers = [])]
mod app {
    use atsamx7x_hal as hal;
    use hal::ehal::watchdog::WatchdogDisable;
    use rtt_target::{rprintln, rtt_init_print};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("init");
        // Disable the watchdog.
        hal::watchdog::Watchdog::new(ctx.device.WDT).disable();

        (Shared {}, Local {}, init::Monotonics())
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");

        loop {}
    }
}
