#![no_main]
#![no_std]

//! USB Device support
pub use usb_device;

mod bus;
use self::bus::UsbBus;

mod devicedesc;
use self::devicedesc::Descriptors;
