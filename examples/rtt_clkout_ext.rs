// clkout_ext.rs
//
// Measures the external RC oscillator on PA6
// Connected to EXT2 5 on the XPlained Ultra 71

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

        let pmc = ctx.device.PMC;
        let pioa = ctx.device.PIOA;
        let utmi = ctx.device.UTMI;

        // clock enable to pioa (peripheral identifier 10)
        pmc.pmc_pcer0.write(|w| w.pid10().set_bit());
        while pmc.pmc_pcsr0.read().pid10().bit_is_clear() {}

        // setup main crystal oscillator
        pmc.ckgr_mor.modify(|_, w| {
            w.key().passwd();
            w.moscxtby().clear_bit();
            w.moscxten().set_bit();
            unsafe {
                w.moscxtst().bits(u8::MAX); // 62 ms?
            }
            w
        });

        // Wait until oscillator is stable.
        while pmc.pmc_sr.read().moscxts().bit_is_clear() {}

        // Configure the UTMI PLL clock and wait for lock.
        utmi.utmi_cktrim.modify(|_, w| w.freq().xtal12());
        pmc.ckgr_uckr.modify(|_, w| {
            w.upllen().set_bit();
            unsafe {
                w.upllcount().bits(u8::MAX);
            }
            w
        });
        // Wait until UTMI_PLL is stable.
        while pmc.pmc_sr.read().locku().bit_is_clear() {}

        // Set divider to two
        pmc.pmc_mckr.modify(|_, w| w.uplldiv2().set_bit());

        // master clock with a prescaler 100 (2.4 MHz expected)
        // (HW adds 1 to prescaler value)
        pmc.pmc_pck[0].write(|w| unsafe { w.css().upll_clk().pres().bits(100 - 1) });

        // Enable PCK0.
        pmc.pmc_scer.write(|w| w.pck0().set_bit());

        // Wait for PCK0 ready.
        while pmc.pmc_sr.read().pckrdy0().bit_is_clear() {}

        // Configure pins for alternate function B (0b01), clk0

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
        rprintln!("idle");

        loop {}
    }
}
