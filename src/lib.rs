#![no_main]
#![no_std]

//! USB Device support
pub use usb_device;

mod bus;
pub use self::bus::{usb_allocator, UsbBus};

mod devicedesc;
use devicedesc::Descriptors;
