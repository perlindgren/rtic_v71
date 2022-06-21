//!
//! > cargo embed --example rtt_usb --release
//!
#![no_std]
#![no_main]

use panic_rtt_target as _;

#[rtic::app(device = atsamx7x_hal::target_device, dispatchers = [])]
mod app {

    use x7x_usb::*;

    use atsamx7x_hal as hal;

    use cortex_m::asm;
    use hal::ehal::watchdog::WatchdogDisable;
    use rtt_target::{rprintln, rtt_init_print};
    use usb_device::prelude::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}
    #[init(local = [EP_MEMORY: [u32; 1024] = [0; 1024]])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = cx.device;
        let wdt = dp.WDT;
        // Disable the watchdog.
        hal::watchdog::Watchdog::new(wdt).disable();

        rtt_init_print!();
        rprintln!("rtic: init");

        let pmc = dp.PMC;
        let utmi = dp.UTMI;
        let usb_hs = dp.USBHS;
        let pioa = dp.PIOA;

        // clock enable to usb_sh (peripheral identifier 34)
        pmc.pmc_pcer1.write(|w| w.pid34().set_bit());
        while pmc.pmc_pcsr1.read().pid34().bit_is_clear() {}

        // Setup main crystal oscillator, 31.3
        pmc.ckgr_mor.modify(|_, w| {
            w.key().passwd();
            w.moscxtby().clear_bit(); // don't bypass power to external crystal
            w.moscxten().set_bit(); // external crystal
            unsafe {
                w.moscxtst().bits(u8::MAX); // 62 ms?
            }
            w
        });

        // Wait until oscillator is stable.
        while pmc.pmc_sr.read().moscxts().bit_is_clear() {}

        // Configure the UTMI PLL clock and wait for lock. 30.7
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

        rprintln!("rtic: HS clock init");

        // Setup FS clock, 31.3
        // PMC_MCKR

        // Set divider to two
        pmc.pmc_mckr.modify(|_, w| w.uplldiv2().set_bit());

        // UPLL clock with a prescaler 100 (2.4 MHz expected)
        // (HW adds 1 to prescaler value)
        pmc.pmc_pck[0].write(|w| unsafe { w.css().upll_clk().pres().bits(100 - 1) });

        // setup FS clk

        // use the UPPL
        pmc.pmc_usb.write(|w| {
            w.usbs().set_bit();
            unsafe { w.usbdiv().bits(10) }
        });

        // Wait until UTMI_PLL is stable.
        while pmc.pmc_sr.read().locku().bit_is_clear() {}

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

        let usb_bus = Usbd::new(usb_hs);
        rprintln!("rtic: bus init");

        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(0xff)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();

        rprintln!("rtic: usb device initialized");
        loop {
            rprintln!("rtic: poll");

            let _ = usb_dev.poll(&mut []);

            rprintln!("rtic: wait");
            for _ in 0..10_000 {
                asm::nop();
            }
        }
        #[allow(unreachable_code)]
        (Shared {}, Local {}, init::Monotonics())
    }
}
