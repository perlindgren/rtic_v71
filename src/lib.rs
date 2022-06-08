#![no_main]
#![no_std]

use usb_device::{
    bus::{PollResult, UsbBus, UsbBusAllocator},
    endpoint::{EndpointAddress, EndpointDirection, EndpointType},
    Result, UsbDirection, UsbError,
};

use bare_metal::Mutex;
use cortex_m_semihosting::hprintln;

mod peripheral_identifiers;
pub use peripheral_identifiers::PeripheralIdentifiers;

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
    xfer_status: [Option<XferStatus>; MAX_ENDPOINTS],
}

// Transfer control context

#[derive(Debug, Copy, Clone)]
struct XferStatus {
    //uint8_t * buffer;
    // uint16_t total_len;
    // uint16_t queued_len;
    max_packet_size: u16,
    // uint8_t interval;
    // tu_fifo_t * fifo;
}

const MAX_ENDPOINTS: usize = 10;

impl Usbd {
    /// Creates a new USB bus, taking ownership of the raw peripheral.
    ///
    /// # Parameters
    ///
    /// * `periph`: The raw USBD peripheral. (assume initialized)
    #[inline]
    pub fn new(usb_hs: pac::USBHS) -> UsbBusAllocator<Self> {
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
            xfer_status: [None; MAX_ENDPOINTS],
        })
    }
}

impl UsbBus for Usbd {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        mut ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        ep_max_packet_size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        hprintln!(
            "alloc_ep: dir {:?}, addr {:?}, type {:?}, max_size {:?}",
            ep_dir,
            ep_addr,
            ep_type,
            ep_max_packet_size
        );

        // create end point address if None
        // not sure if we should allow automatic allocation of ep0?
        // (right now auto allocation starts from ep1)
        if ep_addr.is_none() {
            ep_addr = Some(EndpointAddress::from_parts(
                match self.xfer_status[1..]
                    .iter()
                    .enumerate()
                    .find(|(_, x)| x.is_none())
                {
                    Some((index, _)) => index + 1,
                    None => return Err(UsbError::EndpointOverflow),
                },
                ep_dir,
            ));
        }

        hprintln!(
            "alloc_ep: dir {:?}, addr {:?}, type {:?}, max_size {:?}",
            ep_dir,
            ep_addr,
            ep_type,
            ep_max_packet_size
        );

        // for now assume ep_addr set
        let ep_index = ep_addr.unwrap().index();
        hprintln!("ep_index {}", ep_index);

        // check endpoint address, only 0..9 allowed
        if ep_index > 9 || ep_max_packet_size > 1024 {
            hprintln!("invalid index or size");
            return Err(UsbError::InvalidEndpoint);
        }

        // Safety: Usbd owns the USBHS
        let usb_hs = unsafe { &*pac::USBHS::ptr() };

        self.xfer_status[ep_index] = Some(XferStatus {
            max_packet_size: ep_max_packet_size,
        });

        // Per: not sure if the below is needed
        // Note to self: Depends how the actual reset/allocation is done in HW
        // USB_REG->DEVEPT &=~(1 << (DEVEPT_EPRST0_Pos + epnum));

        // reset endpoint
        hprintln!("reset {:x}", usb_hs.usbhs_devept.read().bits());

        usb_hs.usbhs_devept.modify(|_, w| match ep_index {
            0 => w.eprst0().set_bit(),
            1 => w.eprst1().set_bit(),
            2 => w.eprst2().set_bit(),
            3 => w.eprst3().set_bit(),
            4 => w.eprst4().set_bit(),
            5 => w.eprst5().set_bit(),
            6 => w.eprst6().set_bit(),
            7 => w.eprst7().set_bit(),
            8 => w.eprst8().set_bit(),
            9 => w.eprst9().set_bit(),

            _ => unreachable!(),
        });

        hprintln!("reset {:x}", usb_hs.usbhs_devept.read().bits());

        // enable endpoint
        usb_hs.usbhs_devept.modify(|_, w| match ep_index {
            0 => w.epen0().set_bit(),
            1 => w.epen1().set_bit(),
            2 => w.epen2().set_bit(),
            3 => w.epen3().set_bit(),
            4 => w.epen4().set_bit(),
            5 => w.epen5().set_bit(),
            6 => w.epen6().set_bit(),
            7 => w.epen7().set_bit(),
            8 => w.epen8().set_bit(),
            9 => w.epen9().set_bit(),
            _ => unreachable!(),
        });

        hprintln!("reset and enable {:x}", usb_hs.usbhs_devept.read().bits());

        // generic configuration
        usb_hs.usbhs_deveptcfg[ep_index].write(|w| {
            //
            match ep_max_packet_size {
                0..=8 => w.epsize()._8_byte(),
                9..=16 => w.epsize()._16_byte(),
                17..=32 => w.epsize()._32_byte(),
                33..=64 => w.epsize()._64_byte(),
                65..=128 => w.epsize()._128_byte(),
                129..=256 => w.epsize()._256_byte(),
                257..=512 => w.epsize()._512_byte(),
                513..=1024 => w.epsize()._1024_byte(),
                _ => unreachable!(),
            };
            // set bank
            w.epbk()._1_bank();

            // force allocation
            w.alloc().set_bit()
        });

        if ep_index == 0 {
            // Configure the Endpoint 0 configuration register
            usb_hs.usbhs_deveptcfg[0].modify(|_, w| {
                // set end point type to CTRL
                w.eptype().ctrl()
            });
        } else {
            hprintln!("ep {}", ep_index);
            usb_hs.usbhs_deveptcfg[ep_index].modify(|_, w| {
                // set end point type
                w.eptype().bits(ep_type as u8);

                // autosw
                w.autosw().set_bit();

                // set nbtrans
                if ep_type == EndpointType::Isochronous {
                    w.nbtrans()._1_trans();
                }

                // direction
                w.epdir().bit(ep_dir == UsbDirection::Out)
            });

            // todo, dual bank
        };
        // svd2rust API distinguish modes, we use ctrl mode here
        // to emulate the "unsafe" use from the TinyUsb abstraction for now.

        // setup RSTDTS
        usb_hs.usbhs_deveptier_ctrl_mode()[ep_index].write(|w| w.rstdts().set_bit());

        // setup STALLRQC
        usb_hs.usbhs_deveptidr_ctrl_mode()[ep_index].write(|w| w.stallrqc().set_bit());

        // check that endpoint was correctly initiated
        // Notice, re-allocation might fail in case size is larger, so be aware
        if usb_hs.usbhs_deveptisr_ctrl_mode()[ep_index]
            .read()
            .cfgok()
            .bit_is_set()
        {
            // Endpoint configuration is successful
            usb_hs.usbhs_deveptier_ctrl_mode()[ep_index].write(|w| w.rxstpes().set_bit());
            // Enable Endpoint Interrupt
            usb_hs.usbhs_devier.write(|w| match ep_index {
                0 => w.pep_0().set_bit(),
                1 => w.pep_1().set_bit(),
                2 => w.pep_2().set_bit(),
                3 => w.pep_3().set_bit(),
                4 => w.pep_4().set_bit(),
                5 => w.pep_5().set_bit(),
                6 => w.pep_6().set_bit(),
                7 => w.pep_7().set_bit(),
                8 => w.pep_8().set_bit(),
                9 => w.pep_9().set_bit(),

                _ => unreachable!(),
            });

            Ok(ep_addr.unwrap())
        } else {
            hprintln!("endpoint failed");
            Err(UsbError::InvalidEndpoint)
        }
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
