#![no_main]
#![no_std]

use core::task::Poll;

use usb_device::{
    bus::{PollResult, UsbBus, UsbBusAllocator},
    endpoint::{EndpointAddress, EndpointDirection, EndpointType},
    Result, UsbDirection, UsbError,
};

use bare_metal::Mutex;
// use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

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
    // status
    connected: AtomicBool,
    addressed: AtomicBool,
    cfg_num: AtomicBool,
    suspended: AtomicBool,
    xfer_status: [Option<XferStatus>; MAX_ENDPOINTS],
}

#[derive(Debug)]
enum EventType {
    Unplugged,
    Suspend,
    Resume,
    Sof,
}

#[derive(Debug)]
struct Event {
    event_type: EventType,
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
            connected: AtomicBool::new(false),
            addressed: AtomicBool::new(false),
            cfg_num: AtomicBool::new(false),
            suspended: AtomicBool::new(false),
            xfer_status: [None; MAX_ENDPOINTS],
        })
    }

    fn event_handler(&self, event: Event) {
        match event.event_type {
            EventType::Unplugged => {
                rprintln!("event_handler: unplugged");
                self.connected.store(false, Ordering::Release);
                self.addressed.store(false, Ordering::Release);
                self.cfg_num.store(false, Ordering::Release);
                self.suspended.store(false, Ordering::Release);
            }

            EventType::Suspend => {
                rprintln!("event_handler: suspend");
                // NOTE: When plugging/unplugging device, the D+/D- state are unstable and
                // can accidentally meet the SUSPEND condition ( Bus Idle for 3ms ).
                // In addition, some MCUs such as SAMD or boards that haven no VBUS detection cannot distinguish
                // suspended vs disconnected. We will skip handling SUSPEND/RESUME event if not currently connected
                if self.connected.load(Ordering::Acquire) {
                    self.suspended.store(true, Ordering::Release);
                    //         osal_queue_send(_usbd_q, event, in_isr);
                }
            }
            EventType::Resume => {
                rprintln!("event_handler: resume");
                // skip event if not connected (especially required for SAMD)
                if self.connected.load(Ordering::Acquire) {
                    self.suspended.store(false, Ordering::Release);
                    //         osal_queue_send(_usbd_q, event, in_isr);
                }
            }
            EventType::Sof => {
                rprintln!("event_handler: sof");
                // Some MCUs after running dcd_remote_wakeup() does not have way to detect the end of remote wakeup
                // which last 1-15 ms. DCD can use SOF as a clear indicator that bus is back to operational
                if self.suspended.load(Ordering::Acquire) {
                    self.suspended.store(false, Ordering::Release);
                    //         dcd_event_t const event_resume = { .rhport = event->rhport, .event_id = DCD_EVENT_RESUME };
                    //         osal_queue_send(_usbd_q, &event_resume, in_isr);
                }
            } // default:
              //       osal_queue_send(_usbd_q, event, in_isr);
              //     break;
        }
    }

    fn get_reg(&self) -> &pac::usbhs::RegisterBlock {
        // Safety: Usbd owns the USBHS
        unsafe { &*pac::USBHS::ptr() }
    }

    fn reset_ep(&self, ep_index: usize) {
        let usb_hs = self.get_reg();
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

        usb_hs.usbhs_devept.modify(|_, w| match ep_index {
            0 => w.eprst0().clear_bit(),
            1 => w.eprst1().clear_bit(),
            2 => w.eprst2().clear_bit(),
            3 => w.eprst3().clear_bit(),
            4 => w.eprst4().clear_bit(),
            5 => w.eprst5().clear_bit(),
            6 => w.eprst6().clear_bit(),
            7 => w.eprst7().clear_bit(),
            8 => w.eprst8().clear_bit(),
            9 => w.eprst9().clear_bit(),

            _ => unreachable!(),
        });
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
            "usb-device: alloc_ep: dir {:?}, addr {:?}, type {:?}, max_size {:?}",
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

        self.xfer_status[ep_index] = Some(XferStatus {
            max_packet_size: ep_max_packet_size,
        });

        // Safety: Usbd owns the USBHS
        let usb_hs = self.get_reg();

        // Per: not sure if the below is needed
        // Note to self: Depends how the actual reset/allocation is done in HW
        // USB_REG->DEVEPT &=~(1 << (DEVEPT_EPRST0_Pos + epnum));

        // reset endpoint
        self.reset_ep(ep_index);

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

                // direction,
                // 0 (OUT): The endpoint direction is OUT.
                // 1 (IN): The endpoint direction is IN (nor for control endpoints).
                w.epdir().bit(ep_dir == UsbDirection::In);

                // autosw, Per: do we really need this if not supporting multiple banks
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

    // Enables and initializes the USB peripheral.
    // Soon after enabling the device will be reset, so there is no need to perform a USB reset in this method.
    fn enable(&mut self) {
        rprintln!("usb-device: enable {:?}", self);

        // Per: Not sure if this should go here
        // For now we setup the usb device in `init`
        //
        //   (void) rhport;
        //   dcd_int_disable(rhport);
        //   // Enable the USB controller in device mode
        //   USB_REG->CTRL = CTRL_UIMOD | CTRL_USBE;
        //   while (!(USB_REG->SR & SR_CLKUSABLE));
        //   #if TUD_OPT_HIGH_SPEED
        //     USB_REG->DEVCTRL &= ~DEVCTRL_SPDCONF;
        //   #else
        //     USB_REG->DEVCTRL |= DEVCTRL_SPDCONF_LOW_POWER;
        //   #endif
        //   // Enable the End Of Reset, Suspend & Wakeup interrupts
        //   USB_REG->DEVIER = (DEVIER_EORSTES | DEVIER_SUSPES | DEVIER_WAKEUPES);
        //   #if USE_SOF
        //     USB_REG->DEVIER = DEVIER_SOFES;
        //   #endif
        //   // Clear the End Of Reset, SOF & Wakeup interrupts
        //   USB_REG->DEVICR = (DEVICR_EORSTC | DEVICR_SOFC | DEVICR_WAKEUPC);
        //   // Manually set the Suspend Interrupt
        //   USB_REG->DEVIFR |= DEVIFR_SUSPS;
        //   // Ack the Wakeup Interrupt
        //   USB_REG->DEVICR = DEVICR_WAKEUPC;
        //   // Attach the device
        //   USB_REG->DEVCTRL &= ~DEVCTRL_DETACH;
        //   // Freeze USB clock
        //   USB_REG->CTRL |= CTRL_FRZCLK;

        // For now only clear detach here
        // Essentially attaching the device
        // Safety: Usbd owns the USBHS
        let usb_hs = self.get_reg();
        usb_hs.usbhs_devctrl.modify(|_, w| w.detach().clear_bit());
    }

    // Called when the host resets the device. This will be soon called after poll returns PollResult::Reset.
    // This method should reset the state of all endpoints and peripheral flags back to a state suitable for enumeration,
    // as well as ensure that all endpoints previously allocated with alloc_ep are initialized as specified.
    fn reset(&self) {
        rprintln!("usb-device: reset {:?}", self);
        // for now just reset the ctrl endpoint
        // not sure if it should be here, or when EORST is detected
        self.reset_ep(0);
        let usb_hs = self.get_reg();
        rprintln!(
            "ep0 is enabled? {:?}",
            usb_hs.usbhs_devept.read().epen0().bit_is_set()
        );

        // we should
    }

    fn set_device_address(&self, addr: u8) {
        rprintln!("usb-device: set_device_address {:?}", self);
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        rprintln!("usb-device: write {:?}", self);
        rprintln!("ep_addr {:?}", ep_addr);
        rprintln!("buf {:?}", buf);

        Err(UsbError::Unsupported)
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        rprintln!("usb-device: write {:?}", self);
        rprintln!("ep_addr {:?}", ep_addr);
        rprintln!("buf {:?}", buf);

        Err(UsbError::Unsupported)
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        rprintln!("usb-device: set_stalled {:?}, {:?}", self, stalled);
        rprintln!("ep_addr {:?}", ep_addr);
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        rprintln!("usb-device: is_stalled {:?}", self);
        rprintln!("ep_addr {:?}", ep_addr);
        false
    }

    // Causes the USB peripheral to enter USB suspend mode, lowering power consumption and preparing to detect a USB wakeup event.
    // This will be called after poll returns PollResult::Suspend.
    // The device will continue be polled, and it shall return a value other than Suspend from poll when it no longer detects the suspend condition.
    fn suspend(&self) {
        rprintln!("usb-device: suspend {:?}", self);
    }

    fn resume(&self) {
        rprintln!("usb-device: resume {:?}", self);
    }

    fn poll(&self) -> PollResult {
        rprintln!("usb-device: poll {:?}", self);

        // Safety: Usbd owns the USBHS
        let usb_hs = unsafe { &*pac::USBHS::ptr() };

        let dev_isr = usb_hs.usbhs_devisr.read();
        rprintln!("dev_irs : {:#010x}", dev_isr.bits());

        if dev_isr.eorst().bit_is_set() {
            rprintln!("eorst");

            // USB_REG->DEVICR = DEVICR_EORSTC;
            // USB_REG->DEVICR = DEVICR_WAKEUPC;
            // USB_REG->DEVICR = DEVICR_SUSPC;
            // USB_REG->DEVIER = DEVIER_SUSPES;

            // for now just reset the ctrl endpoint
            self.reset_ep(0);

            // clear the eorst interrupt
            usb_hs.usbhs_devicr.write(|w| w.eorstc().set_bit());

            // clear the wakeup interrupt
            usb_hs.usbhs_devicr.write(|w| w.wakeupc().set_bit());

            // clear the wakeup interrupt
            usb_hs.usbhs_devicr.write(|w| w.suspc().set_bit());

            // enable the wakeup interrupt
            usb_hs.usbhs_devier.write(|w| w.wakeupes().set_bit());

            self.event_handler(Event {
                event_type: EventType::Suspend,
            });
            return PollResult::Reset;
        }

        if dev_isr.eorsm().bit_is_set() {
            rprintln!("eorsm")
        }

        if dev_isr.msof().bit_is_set() {
            rprintln!("msof")
        }

        if dev_isr.pep_0().bit_is_set() {
            rprintln!("pep_0")
        }

        if dev_isr.sof().bit_is_set() {
            rprintln!("sof")
        }

        if dev_isr.susp().bit_is_set() {
            rprintln!("susp");
            // Unfreeze USB clock
            // USB_REG->CTRL &= ~CTRL_FRZCLK;
            // while (!(USB_REG->SR & SR_CLKUSABLE));
            // USB_REG->DEVICR = DEVICR_SUSPC;
            // USB_REG->DEVIDR = DEVIDR_SUSPEC;
            // USB_REG->DEVIER = DEVIER_WAKEUPES;
            // USB_REG->CTRL |= CTRL_FRZCLK;

            // dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);

            // Per: for now we let the usb be clocked, just handle the HW events
            // clear the susp interrupt
            usb_hs.usbhs_devicr.write(|w| w.suspc().set_bit());
            // disable the susp interrupt
            usb_hs.usbhs_devidr.write(|w| w.suspec().set_bit());
            // enable the wakeup interrupt
            usb_hs.usbhs_devier.write(|w| w.wakeupes().set_bit());
            self.event_handler(Event {
                event_type: EventType::Suspend,
            });
        }

        if dev_isr.uprsm().bit_is_set() {
            rprintln!("uprsm")
        }

        if dev_isr.wakeup().bit_is_set() {
            rprintln!("wakeup");
            // USB_REG->DEVICR = DEVICR_WAKEUPC;
            // USB_REG->DEVIDR = DEVIDR_WAKEUPEC;
            // USB_REG->DEVIER = DEVIER_SUSPES;

            // dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);

            // clear the wakeup interrupt
            usb_hs.usbhs_devicr.write(|w| w.wakeupc().set_bit());
            // disable wakeup the interrupt
            usb_hs.usbhs_devidr.write(|w| w.wakeupec().set_bit());
            // enable the susp interrupt
            usb_hs.usbhs_devier.write(|w| w.suspes().set_bit());
            // event

            // dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
            // dcd_event_t event = { .rhport = rhport, .event_id = eid };
            // dcd_event_handler(&event, in_isr);
            self.event_handler(Event {
                event_type: EventType::Resume,
            })
        }

        // Endpoints interrupt
        //   for (int ep_ix = 0; ep_ix < EP_MAX; ep_ix++)
        //   {
        //     if (int_status & (DEVISR_PEP_0 << ep_ix))
        //     {
        //       dcd_ep_handler(ep_ix);
        //     }
        //   }

        if usb_hs.usbhs_deveptisr_ctrl_mode()[0]
            .read()
            .rxstpi()
            .bit_is_set()
        {
            rprintln!("------------------------------------------------------------ here --------------------------")
        };

        // a bit ugly
        if dev_isr.pep_0().bit_is_set() {
            rprintln!("pep0");
        }
        if dev_isr.pep_1().bit_is_set() {
            rprintln!("pep1");
        }
        if dev_isr.pep_2().bit_is_set() {
            rprintln!("pep2");
        }
        if dev_isr.pep_3().bit_is_set() {
            rprintln!("pep3");
        }
        if dev_isr.pep_4().bit_is_set() {
            rprintln!("pep4");
        }
        if dev_isr.pep_5().bit_is_set() {
            rprintln!("pep5");
        }
        if dev_isr.pep_6().bit_is_set() {
            rprintln!("pep6");
        }
        if dev_isr.pep_7().bit_is_set() {
            rprintln!("pep7");
        }
        if dev_isr.pep_8().bit_is_set() {
            rprintln!("pep8");
        }
        if dev_isr.pep_9().bit_is_set() {
            rprintln!("pep9");
        }

        //USB_REG->DEVISR;

        // uint32_t int_status = USB_REG->DEVEPTISR[ep_ix];
        // int_status &= USB_REG->DEVEPTIMR[ep_ix];

        // uint16_t count = (USB_REG->DEVEPTISR[ep_ix] &
        //                   DEVEPTISR_BYCT) >> DEVEPTISR_BYCT_Pos;
        // xfer_ctl_t *xfer = &xfer_status[ep_ix];
        PollResult::None
    }
}

// void dcd_int_handler(uint8_t rhport)
// {
//   (void) rhport;
//   uint32_t int_status = USB_REG->DEVISR;
//   int_status &= USB_REG->DEVIMR;
//   // End of reset interrupt
//   if (int_status & DEVISR_EORST)
//   {
//     // Unfreeze USB clock
//     USB_REG->CTRL &= ~CTRL_FRZCLK;
//     while(!(USB_REG->SR & SR_CLKUSABLE));
//     // Reset all endpoints
//     for (int ep_ix = 1; ep_ix < EP_MAX; ep_ix++)
//     {
//       USB_REG->DEVEPT |= 1 << (DEVEPT_EPRST0_Pos + ep_ix);
//       USB_REG->DEVEPT &=~(1 << (DEVEPT_EPRST0_Pos + ep_ix));
//     }
//     dcd_edpt_open (0, &ep0_desc);
//     USB_REG->DEVICR = DEVICR_EORSTC;
//     USB_REG->DEVICR = DEVICR_WAKEUPC;
//     USB_REG->DEVICR = DEVICR_SUSPC;
//     USB_REG->DEVIER = DEVIER_SUSPES;

//     dcd_event_bus_reset(rhport, get_speed(), true);
//   }
//   // End of Wakeup interrupt
//   if (int_status & DEVISR_WAKEUP)
//   {
//     USB_REG->CTRL &= ~CTRL_FRZCLK;
//     while (!(USB_REG->SR & SR_CLKUSABLE));
//     USB_REG->DEVICR = DEVICR_WAKEUPC;
//     USB_REG->DEVIDR = DEVIDR_WAKEUPEC;
//     USB_REG->DEVIER = DEVIER_SUSPES;

//     dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
//   }
//   // Suspend interrupt
//   if (int_status & DEVISR_SUSP)
//   {
//     // Unfreeze USB clock
//     USB_REG->CTRL &= ~CTRL_FRZCLK;
//     while (!(USB_REG->SR & SR_CLKUSABLE));
//     USB_REG->DEVICR = DEVICR_SUSPC;
//     USB_REG->DEVIDR = DEVIDR_SUSPEC;
//     USB_REG->DEVIER = DEVIER_WAKEUPES;
//     USB_REG->CTRL |= CTRL_FRZCLK;

//     dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
//   }
// #if USE_SOF
//   if(int_status & DEVISR_SOF)
//   {
//     USB_REG->DEVICR = DEVICR_SOFC;

//     dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
//   }
// #endif
//   // Endpoints interrupt
//   for (int ep_ix = 0; ep_ix < EP_MAX; ep_ix++)
//   {
//     if (int_status & (DEVISR_PEP_0 << ep_ix))
//     {
//       dcd_ep_handler(ep_ix);
//     }
//   }
//   // Endpoints DMA interrupt
//   for (int ep_ix = 0; ep_ix < EP_MAX; ep_ix++)
//   {
//     if (EP_DMA_SUPPORT(ep_ix))
//     {
//       if (int_status & (DEVISR_DMA_1 << (ep_ix - 1)))
//       {
//         dcd_dma_handler(ep_ix);
//       }
//     }
//   }
// }

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
