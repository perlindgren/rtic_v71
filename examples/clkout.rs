// clkout.rs
//
// Measures the internal RC oscillator on PA6
// Connected to EXT2 5 on the XPlained Ultra 71

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
        let pioa = ctx.device.PIOA;

        // clock enable to pioa (peripheral identifier 10)
        pmc.pmc_pcer0.write(|w| w.pid10().set_bit());
        while pmc.pmc_pcsr0.read().pid10().bit_is_clear() {}

        // Clock setup for Master/128
        // disable clk0
        pmc.pmc_scdr.write(|w| w.pck0().set_bit());

        // master clock with a prescaler 128
        // (HW adds 1 to prescaler value)
        pmc.pmc_pck[0].write(|w| unsafe { w.css().mck().pres().bits(127) });

        // Enable PCK0.
        pmc.pmc_scer.write(|w| w.pck0().set_bit());

        // Wait for PCK0 ready.
        while pmc.pmc_sr.read().pckrdy0().bit_is_clear() {}

        // Configure pins for function B, clk0
        pioa.pio_abcdsr[1].modify(|_, w| w.p6().clear_bit());
        pioa.pio_abcdsr[0].modify(|_, w| w.p6().set_bit());

        // Give pins to the peripheral.
        pioa.pio_pdr.write(|w| w.p6().set_bit());
        cortex_m::asm::dsb();

        // disable multidrive
        pioa.pio_mddr.write(|w| w.p6().set_bit());
        cortex_m::asm::dsb();

        // ensure we don't pull the pni up/down
        pioa.pio_pudr.write(|w| w.p6().set_bit());
        pioa.pio_ppddr.write(|w| w.p6().set_bit());
        cortex_m::asm::dsb();

        (Shared {}, Local {}, init::Monotonics())
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle");

        loop {}
    }
}
