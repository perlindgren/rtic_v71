// led.rs
//
// Blinks the LED1 on the XPlained Ultra, PC09 (4.3.3)
// http://ww1.microchip.com/downloads/en/devicedoc/atmel-42408-samv71-xplained-ultra_user-guide.pdf

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

        let pmc = ctx.device.PMC;
        let pioc = ctx.device.PIOC;

        // power to pioc (peripheral identifier 12)
        pmc.pmc_pcer0.write(|w| w.pid12().set_bit());
        while pmc.pmc_pcsr0.read().pid12().bit_is_clear() {}

        // Configure PC09 as general purpose output
        // after reset the pin is controlled by the pio
        // 32.5.2
        {
            // output enable
            pioc.pio_oer.write(|w| w.p9().set_bit());
            loop {
                // set output
                pioc.pio_sodr.write(|w| w.p9().set_bit());
                // clear output
                pioc.pio_codr.write(|w| w.p9().set_bit());
            }
        }

        (Shared {}, Local {}, init::Monotonics())
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle");

        loop {}
    }
}
