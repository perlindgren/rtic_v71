//!
//! //! CDC-ACM serial port example using polling in a busy loop.
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
        rprintln!("init");

        let pmc = dp.PMC;
        let utmi = dp.UTMI;
        let usb_hs = dp.USBHS;

        // clock enable to usb_sh (peripheral identifier 34)
        pmc.pmc_pcer1.write(|w| w.pid34().set_bit());
        while pmc.pmc_pcsr1.read().pid34().bit_is_clear() {}

        // 39.5.2
        usb_hs.usbhs_ctrl.write(|w| {
            w.usbe().set_bit(); // enable usb_hs
            w.uimod().set_bit(); // enable device mode
            w.frzclk().clear_bit() // enable clocking
        });

        // wait until clock stable
        // while usb_hs.usbhs_sr.read().clkusable().bit_is_clear() {}

        // enable interrupts
        usb_hs.usbhs_devier.write(|w| {
            w.eorstes().set_bit();
            w.suspes().set_bit();
            w.wakeupes().set_bit();
            w.sofes().set_bit()
        });

        // clear interrupts
        usb_hs.usbhs_devicr.write(|w| {
            w.eorstc().set_bit();
            w.sofc().set_bit();
            w.wakeupc().set_bit()
        });

        // Manually set the Suspend Interrupt
        usb_hs.usbhs_devifr.write(|w| w.susps().set_bit());

        // check mode, probably not valid unless reset by host
        rprintln!(
            "speed {:?}",
            usb_hs.usbhs_devctrl.read().spdconf().is_high_speed()
        );

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

        // Per: Note
        // Here we might want to setup the USB_FS clock as well

        rprintln!("clock init");

        let usb_bus = Usbd::new(usb_hs);
        rprintln!("bus init");

        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(0xff)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();

        rprintln!("usb device");
        loop {
            let _ = usb_dev.poll(&mut []);

            rprintln!(".");
            for _ in 0..100_000 {
                asm::nop();
            }
        }
        #[allow(unreachable_code)]
        (Shared {}, Local {}, init::Monotonics())
    }
}
