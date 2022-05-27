//!
//! //! CDC-ACM serial port example using polling in a busy loop.
//!
#![no_std]
#![no_main]

use panic_semihosting as _;

#[rtic::app(device = atsamx7x_hal::target_device, dispatchers = [])]
mod app {

    use x7x_usb::*;

    use atsamx7x_hal as hal;
    use cortex_m_semihosting::hprintln;
    use hal::ehal::watchdog::WatchdogDisable;
    use usb_device::prelude::*;
    use usbd_serial::{SerialPort, USB_CLASS_CDC};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}
    #[init(local = [EP_MEMORY: [u32; 1024] = [0; 1024]])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = cx.device;

        let usb_bus = Usbd::new(dp.USBHS);
        // let mut serial = SerialPort::new(&usb_bus);

        // let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        //     .manufacturer("Fake company")
        //     .product("Serial port")
        //     .serial_number("TEST")
        //     .device_class(USB_CLASS_CDC)
        //     .max_packet_size_0(64) // (makes control transfers 8x faster)
        //     .build();

        // loop {
        //     if !usb_dev.poll(&mut [&mut serial]) {
        //         continue;
        //     }

        //     let mut buf = [0u8; 64];

        //     match serial.read(&mut buf) {
        //         Ok(count) if count > 0 => {
        //             // Echo back in upper case
        //             for c in buf[0..count].iter_mut() {
        //                 if 0x61 <= *c && *c <= 0x7a {
        //                     *c &= !0x20;
        //                 }
        //             }

        //             let mut write_offset = 0;
        //             while write_offset < count {
        //                 match serial.write(&buf[write_offset..count]) {
        //                     Ok(len) if len > 0 => {
        //                         write_offset += len;
        //                     }
        //                     _ => {}
        //                 }
        //             }
        //         }
        //         _ => {}
        //     }
        // }

        (Shared {}, Local {}, init::Monotonics())
    }
}
