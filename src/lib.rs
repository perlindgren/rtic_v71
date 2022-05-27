#![no_main]
#![no_std]

use usb_device::{
    bus::{PollResult, UsbBus, UsbBusAllocator},
    endpoint::{EndpointAddress, EndpointType},
    Result, UsbDirection,
};

use bare_metal::Mutex;

use atsamx7x_hal::target_device as pac;

// pub struct UsbPeripheral {
//     _usbhs: pac::USBHS,
// }

// impl UsbPeripheral {
//     pub fn new(usbhs: pac::USBHS) -> Self {
//         Self { _usbhs: usbhs }
//     }
// }

/// A trait for device-specific USB peripherals. Implement this to add support for a new hardware
/// platform. Peripherals that have this trait must have the same register block as NRF52 USBD
/// peripherals.
// pub unsafe trait UsbPeripheralTrait: Send {
//     /// Pointer to the register block
//     const REGISTERS: *const ();
// }

// unsafe impl UsbPeripheralTrait for UsbPeripheral {
//     const REGISTERS: *const () = pac::USBHS::ptr() as *const _;
// }

/// USB device implementation.
pub struct Usbd {
    // argument passed to `UsbDeviceBuilder.max_packet_size_0`
    max_packet_size_0: u16,
    // bufs: Buffers,
    used_in: u8,
    used_out: u8,
    iso_in_used: bool,
    iso_out_used: bool,
    // ep0_state: Mutex<Cell<EP0State>>,
    // busy_in_endpoints: Mutex<Cell<u16>>,
}

impl Usbd {
    /// Creates a new USB bus, taking ownership of the raw peripheral.
    ///
    /// # Parameters
    ///
    /// * `periph`: The raw USBD peripheral.
    #[inline]
    pub fn new(periph: pac::USBHS) -> UsbBusAllocator<Self> {
        UsbBusAllocator::new(Self {
            max_packet_size_0: 0,
            // bufs: Buffers::new(),
            used_in: 0,
            used_out: 0,
            iso_in_used: false,
            iso_out_used: false,
            // ep0_state: Mutex::new(Cell::new(EP0State {
            //     direction: UsbDirection::Out,
            //     remaining_size: 0,
            //     in_transfer_state: TransferState::NoTransfer,
            //     is_set_address: false,
            // })),
            // busy_in_endpoints: Mutex::new(Cell::new(0)),
        })
    }
}

impl UsbBus for Usbd {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        unimplemented!()
    }

    fn enable(&mut self) {
        unimplemented!()
    }
    fn reset(&self) {
        unimplemented!()
    }
    fn set_device_address(&self, addr: u8) {
        unimplemented!()
    }
    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        unimplemented!()
    }
    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        unimplemented!()
    }
    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        unimplemented!()
    }
    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        unimplemented!()
    }
    fn suspend(&self) {
        unimplemented!()
    }
    fn resume(&self) {
        unimplemented!()
    }
    fn poll(&self) -> PollResult {
        unimplemented!()
    }
}

// struct USB {}

// pub struct UsbB<USB> {
//     peripheral: USB,
//     //     regs: Mutex<UsbRegisters>,
//     //     allocator: EndpointAllocator<USB>,
// }

// /// USB peripheral driver

// impl<USB: UsbBus> UsbB<USB> {
//     /// Constructs a new USB peripheral driver.
//     pub fn new(peripheral: USB, ep_memory: &'static mut [u32]) -> UsbBusAllocator<Self> {
//         //         let bus = UsbBus {
//         //             peripheral,
//         //             // regs: Mutex::new(UsbRegisters::new::<USB>()),
//         //             // allocator: EndpointAllocator::new(ep_memory),
//         //         };

//         //         UsbBusAllocator::new(bus)
//         unimplemented!()
//     }
// }

// impl<USB: UsbBus> UsbBus for UsbB<USB> {
//     const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = false;

//     fn alloc_ep(
//         &mut self,
//         ep_dir: UsbDirection,
//         ep_addr: Option<EndpointAddress>,
//         ep_type: EndpointType,
//         max_packet_size: u16,
//         interval: u8,
//     ) -> Result<EndpointAddress> {
//         unimplemented!()
//     }

//     fn enable(&mut self) {
//         unimplemented!()
//     }
//     fn reset(&self) {
//         unimplemented!()
//     }
//     fn set_device_address(&self, addr: u8) {
//         unimplemented!()
//     }
//     fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
//         unimplemented!()
//     }
//     fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
//         unimplemented!()
//     }
//     fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
//         unimplemented!()
//     }
//     fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
//         unimplemented!()
//     }
//     fn suspend(&self) {
//         unimplemented!()
//     }
//     fn resume(&self) {
//         unimplemented!()
//     }
//     fn poll(&self) -> PollResult {
//         unimplemented!()
//     }
// }

// impl UsbBus for USB {
//     const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = false;

//     fn alloc_ep(
//         &mut self,
//         ep_dir: UsbDirection,
//         ep_addr: Option<EndpointAddress>,
//         ep_type: EndpointType,
//         max_packet_size: u16,
//         interval: u8,
//     ) -> Result<EndpointAddress> {
//         unimplemented!()
//     }

//     fn enable(&mut self) {
//         unimplemented!()
//     }
//     fn reset(&self) {
//         unimplemented!()
//     }
//     fn set_device_address(&self, addr: u8) {
//         unimplemented!()
//     }
//     fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
//         unimplemented!()
//     }
//     fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
//         unimplemented!()
//     }
//     fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
//         unimplemented!()
//     }
//     fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
//         unimplemented!()
//     }
//     fn suspend(&self) {
//         unimplemented!()
//     }
//     fn resume(&self) {
//         unimplemented!()
//     }
//     fn poll(&self) -> PollResult {
//         unimplemented!()
//     }

//     // Has default implementation
//     // fn force_reset(&self) -> Result<()> { ... }
// }
