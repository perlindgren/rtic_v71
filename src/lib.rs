#![no_main]
#![no_std]

use core::task::Poll;

use usb_device::{
    bus::{PollResult, UsbBus, UsbBusAllocator},
    endpoint::{EndpointAddress, EndpointDirection, EndpointType},
    Result, UsbDirection, UsbError,
};

use bare_metal::Mutex;
use rtt_target::rprintln;

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

/// USB device implementation.
#[derive(Debug)]
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
    pub fn new(_usb_hs: pac::USBHS) -> UsbBusAllocator<Self> {
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
        rprintln!(
            "alloc_ep: dir {:?}, addr {:?}, type {:?}, max_size {:?}",
            ep_dir,
            ep_addr,
            ep_type,
            ep_max_packet_size
        );

        // create end point address if None
        if ep_addr.is_none() {
            // find next free endpoint
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

        rprintln!(
            "alloc_ep: dir {:?}, addr {:?}, type {:?}, max_size {:?}",
            ep_dir,
            ep_addr,
            ep_type,
            ep_max_packet_size
        );

        let ep_index = ep_addr.unwrap().index();
        rprintln!("ep_index {}", ep_index);

        // check endpoint address,
        // only 0..=9 allowed
        // ep 0 max size 64
        // ep 1..=9 max size 1024
        if ep_index > 9 || ep_max_packet_size > 1024 || (ep_index == 0 && ep_max_packet_size > 64) {
            rprintln!("invalid index or size");
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
        rprintln!("reset {:x}", usb_hs.usbhs_devept.read().bits());

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

        rprintln!("reset {:x}", usb_hs.usbhs_devept.read().bits());

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

        rprintln!("reset and enable {:x}", usb_hs.usbhs_devept.read().bits());

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

            // direction,
            // 0 (OUT): The endpoint direction is OUT.
            // 1 (IN): The endpoint direction is IN (nor for control endpoints).
            w.epdir().bit(ep_dir == UsbDirection::In);

            // force allocation
            w.alloc().set_bit()
        });

        if ep_index == 0 {
            rprintln!("ep {} - set ctrl", ep_index);
            // ep0 used as ctrl endpoint
            // Configure the Endpoint 0 configuration register
            usb_hs.usbhs_deveptcfg[0].modify(|_, w| {
                // set end point type to CTRL
                w.eptype().ctrl()
            });
        } else {
            rprintln!("ep {} - set {:?}", ep_index, ep_type);
            usb_hs.usbhs_deveptcfg[ep_index].modify(|_, w| {
                // set end point type
                w.eptype().bits(ep_type as u8);

                // autosw
                w.autosw().set_bit();

                // set nbtrans
                if ep_type == EndpointType::Isochronous {
                    w.nbtrans()._1_trans();
                }

                w
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
            rprintln!("endpoint failed");
            Err(UsbError::InvalidEndpoint)
        }
    }

    fn enable(&mut self) {
        rprintln!("enable {:?}", self);
    }

    fn reset(&self) {
        rprintln!("reset {:?}", self);
    }

    fn set_device_address(&self, addr: u8) {
        rprintln!("set_device_address {:?}", self);
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        rprintln!("write {:?}", self);
        rprintln!("ep_addr {:?}", ep_addr);
        rprintln!("buf {:?}", buf);

        Err(UsbError::Unsupported)
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        rprintln!("write {:?}", self);
        rprintln!("ep_addr {:?}", ep_addr);
        rprintln!("buf {:?}", buf);

        Err(UsbError::Unsupported)
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        rprintln!("set_stalled {:?}, {:?}", self, stalled);
        rprintln!("ep_addr {:?}", ep_addr);
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        rprintln!("is_stalled {:?}", self);
        rprintln!("ep_addr {:?}", ep_addr);
        false
    }

    fn suspend(&self) {
        rprintln!("suspend {:?}", self);
    }

    fn resume(&self) {
        rprintln!("resume {:?}", self);
    }

    fn poll(&self) -> PollResult {
        rprintln!("poll {:?}", self);
        PollResult::None
    }
}

// EP_GET_FIFO_PTR(ep, scale) (((TU_XSTRCAT(TU_STRCAT(uint, scale),_t) (*)[0x8000 / ((scale) / 8)])FIFO_RAM_ADDR)[(ep)])
// FIFO_RAM_ADDR     0xA0100000u

// uint16_t len = (uint16_t)(xfer->total_len - xfer->queued_len);
//   if (len)
//   {
//     if (len > xfer->max_packet_size)
//     {
//       len = xfer->max_packet_size;
//     }
//     uint8_t *ptr = EP_GET_FIFO_PTR(ep_ix,8);
//     if(xfer->buffer)
//     {
//       memcpy(ptr, xfer->buffer + xfer->queued_len, len);
//     }
//     else
//     {
//       tu_fifo_read_n(xfer->fifo, ptr, len);
//     }
//     __DSB();
//     __ISB();
//     xfer->queued_len = (uint16_t)(xfer->queued_len + len);
//   }
//   if (ep_ix == 0U)
//   {
//     // Control endpoint: clear the interrupt flag to send the data
//     USB_REG->DEVEPTICR[0] = DEVEPTICR_TXINIC;
//   } else
//   {
//     // Other endpoint types: clear the FIFO control flag to send the data
//     USB_REG->DEVEPTIDR[ep_ix] = DEVEPTIDR_FIFOCONC;
//   }
//   USB_REG->DEVEPTIER[ep_ix] = DEVEPTIER_TXINES;
