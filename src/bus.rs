// This crate is based on the atsam-hal usb implementation.

// This crate uses standard host-centric USB terminology for transfer
// directions. Therefore an OUT transfer refers to a host-to-device transfer,
// and an IN transfer refers to a device-to-host transfer.

use super::Descriptors;

use atsamx7x_hal::target_device as pac;

use crate::devicedesc::DeviceDescBank;
use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::mem;
use cortex_m::interrupt::{self, Mutex};
use cortex_m::singleton;
use usb_device::bus::{PollResult, UsbBusAllocator};
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result as UsbResult, UsbDirection, UsbError};

use rtt_target::rprintln;

pub fn usb_allocator(usb: pac::USBHS) -> UsbBusAllocator<UsbBus> {
    UsbBusAllocator::new(UsbBus::new(usb))
}

/// EPConfig tracks the desired configuration for one side of an endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
struct EPConfig {
    ep_type: EndpointType,
    // allocated_size: u16,
    max_packet_size: u16,
    // addr: usize,
}

impl EPConfig {
    fn new(
        ep_type: EndpointType,
        // allocated_size: u16,
        max_packet_size: u16,
        // buffer_addr: *mut u8,
    ) -> Self {
        Self {
            ep_type,
            // allocated_size,
            max_packet_size,
            // addr: buffer_addr as usize,
        }
    }
}

/// Endpoints tracks the desired configuration of all endpoints managed
/// by the USB peripheral.
///
const MAX_ENDPOINTS: usize = 10;

#[derive(Debug, PartialEq)]
struct Endpoints {
    // None = Disabled
    ep_config: [Option<EPConfig>; 10],
}

impl Endpoints {
    fn new() -> Self {
        Self {
            ep_config: [None; MAX_ENDPOINTS],
        }
    }

    fn find_free_endpoint(&self) -> UsbResult<usize> {
        // start with 1 because 0 is reserved for Control
        for idx in 1..MAX_ENDPOINTS {
            if self.ep_config[idx] == None {
                return Ok(idx);
            }
        }
        Err(UsbError::EndpointOverflow)
    }

    #[allow(clippy::too_many_arguments)]
    fn allocate_endpoint(
        &mut self,
        dir: UsbDirection,
        idx: usize,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> UsbResult<EndpointAddress> {
        if idx != 0 && self.ep_config[idx] != None {
            return Err(UsbError::EndpointOverflow);
        }

        self.ep_config[idx] = Some(EPConfig::new(ep_type, max_packet_size));

        Ok(EndpointAddress::from_parts(idx, dir))
    }
}

struct Inner {
    desc: Descriptors,
    endpoints: Endpoints,
}

pub struct UsbBus {
    inner: Mutex<UnsafeCell<Inner>>,
}

impl UsbBus {
    pub fn new(
        // _clock: &clock::UsbClock,
        // mclk: &mut MCLK,
        // dm_pad: impl AnyPin<Id = PA24>,
        // dp_pad: impl AnyPin<Id = PA25>,
        _usb: pac::USBHS,
    ) -> Self {
        // mclk.ahbmask.modify(|_, w| w.usb_().set_bit());
        // mclk.apbbmask.modify(|_, w| w.usb_().set_bit());

        let inner = Inner {
            // _dm_pad: dm_pad.into().into_mode::<AlternateH>(),
            // _dp_pad: dp_pad.into().into_mode::<AlternateH>(),
            desc: Descriptors::new(),
            endpoints: Endpoints::new(),
        };

        Self {
            inner: Mutex::new(UnsafeCell::new(inner)),
        }
    }
}

// helper functions
impl Inner {
    #[inline(always)]
    fn usbhs(&self) -> &pac::usbhs::RegisterBlock {
        // Safety: Usbd owns the USBHS
        unsafe { &*pac::USBHS::ptr() }
    }

    #[inline(always)]
    fn enable_endpoint(&self, ep_index: usize) {
        self.usbhs().usbhs_devept.modify(|_, w| match ep_index {
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
    }

    #[inline(always)]
    fn enable_endpoint_interrupt(&self, ep_index: usize) {
        self.usbhs().usbhs_devier.write(|w| match ep_index {
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
    }

    // compute fifo address
    #[inline(always)]
    fn fifo_addr(&self, ep_index: usize) -> usize {
        const DPRAM: usize = 0xA0100000;
        DPRAM + ep_index * 0x8000
    }

    // write endpoint fifo
    #[inline(always)]
    fn write_fifo(&self, ep_index: usize, buf: &[u8]) {
        unsafe {
            core::ptr::copy_nonoverlapping(
                buf.as_ptr() as *const u8,
                self.fifo_addr(ep_index) as *mut u8,
                buf.len(),
            );
        }
    }

    // read endpoint fifo
    #[inline(always)]
    fn read_fifo(&self, ep_index: usize, buf: &mut [u8]) {
        unsafe {
            core::ptr::copy_nonoverlapping(
                self.fifo_addr(ep_index) as *const u8,
                buf.as_mut_ptr() as *mut u8,
                buf.len(),
            );
        }
    }

    //     fn set_stall<EP: Into<EndpointAddress>>(&self, ep: EP, stall: bool) {
    //         let ep = ep.into();
    //         if ep.is_out() {
    //             if let Ok(mut bank) = self.bank0(ep) {
    //                 bank.set_stall(stall);
    //             }
    //         } else if let Ok(mut bank) = self.bank1(ep) {
    //             bank.set_stall(stall);
    //         }
    //     }
}

// #[derive(Copy, Clone)]
// enum FlushConfigMode {
//     // Write configuration to all configured endpoints.
//     Full,
//     // Refresh configuration which was reset due to a bus reset.
//     ProtocolReset,
// }

// /// Generate a method that allows returning the endpoint register
// /// for a given endpoint index.  This helps very slightly with
// /// two inconvenient issues:
// /// - the SVD file translation generates a sequence of elements like ecfg0,
// ///   efcg1 rather than an array, so we have to manually translate the indices
// /// - rust doesn't currently have a great solution for generating identifier
// ///   names, so we have to pass in a list of the possible names.
// macro_rules! ep {
//     ($name:ident, $type:ident) => {
//         #[allow(unused)]
//         #[inline]
//         fn $name(&self, endpoint: usize) -> &pac::usb::device::device_endpoint::$type {
//             match endpoint {
//                 0 => &self.usb().device_endpoint0.$name,
//                 1 => &self.usb().device_endpoint1.$name,
//                 2 => &self.usb().device_endpoint2.$name,
//                 3 => &self.usb().device_endpoint3.$name,
//                 4 => &self.usb().device_endpoint4.$name,
//                 5 => &self.usb().device_endpoint5.$name,
//                 6 => &self.usb().device_endpoint6.$name,
//                 7 => &self.usb().device_endpoint7.$name,
//                 _ => unreachable!(),
//             }
//         }
//     };
// }

// struct Bank<'a, T> {
//     address: EndpointAddress,
//     usb: &'a DEVICE,
//     desc: RefMut<'a, super::Descriptors>,
//     _phantom: PhantomData<T>,
//     endpoints: Ref<'a, AllEndpoints>,
// }

// impl<'a, T> Bank<'a, T> {
//     fn usb(&self) -> &DEVICE {
//         self.usb
//     }

//     #[inline]
//     fn index(&self) -> usize {
//         self.address.index()
//     }

//     #[inline]
//     fn config(&mut self) -> &EPConfig {
//         let ep = &self.endpoints.endpoints[self.address.index()];
//         if self.address.is_out() {
//             &ep.bank0
//         } else {
//             &ep.bank1
//         }
//     }
// }

// /// InBank represents In direction banks, Bank #1
// struct InBank;

// /// OutBank represents Out direction banks, Bank #0
// struct OutBank;

// impl<'a> Bank<'a, InBank> {
//     fn desc_bank(&mut self) -> &mut DeviceDescBank {
//         let idx = self.index();
//         self.desc.bank(idx, 1)
//     }

//     /// Returns true if Bank 1 is Ready and thus has data that can be written
//     #[inline]
//     fn is_ready(&self) -> bool {
//         self.epstatus(self.index()).read().bk1rdy().bit()
//     }

//     /// Set Bank 1 Ready.
//     /// Ready means that the buffer contains data that can be sent.
//     #[inline]
//     fn set_ready(&self, ready: bool) {
//         if ready {
//             self.epstatusset(self.index())
//                 .write(|w| w.bk1rdy().set_bit());
//         } else {
//             self.epstatusclr(self.index())
//                 .write(|w| w.bk1rdy().set_bit());
//         }
//     }

//     /// Acknowledges the signal that the last packet was sent.
//     #[inline]
//     fn clear_transfer_complete(&self) {
//         // Clear bits in epintflag by writing them to 1
//         self.epintflag(self.index())
//             .write(|w| w.trcpt1().set_bit().trfail1().set_bit());
//     }

//     /// Indicates if a transfer is complete or pending.
//     #[inline]
//     fn is_transfer_complete(&self) -> bool {
//         self.epintflag(self.index()).read().trcpt1().bit()
//     }

//     /// Writes out endpoint configuration to its in-memory descriptor.
//     fn flush_config(&mut self) {
//         let config = *self.config();
//         {
//             let desc = self.desc_bank();
//             desc.set_address(config.addr as *mut u8);
//             desc.set_endpoint_size(config.max_packet_size);
//             desc.set_multi_packet_size(0);
//             desc.set_byte_count(0);
//         }
//     }

//     /// Enables endpoint-specific interrupts.
//     fn setup_ep_interrupts(&mut self) {
//         self.epintenset(self.index())
//             .write(|w| w.trcpt1().set_bit());
//     }

//     /// Prepares to transfer a series of bytes by copying the data into the
//     /// bank1 buffer. The caller must call set_ready() to finalize the
//     /// transfer.
//     pub fn write(&mut self, buf: &[u8]) -> UsbResult<usize> {
//         let size = buf.len().min(self.config().allocated_size as usize);
//         let desc = self.desc_bank();

//         unsafe {
//             buf.as_ptr()
//                 .copy_to_nonoverlapping(desc.get_address(), size);
//         }

//         desc.set_multi_packet_size(0);
//         desc.set_byte_count(size as u16);

//         Ok(size)
//     }

//     fn is_stalled(&self) -> bool {
//         self.epintflag(self.index()).read().stall1().bit()
//     }

//     fn set_stall(&mut self, stall: bool) {
//         if stall {
//             self.epstatusset(self.index())
//                 .write(|w| w.stallrq1().set_bit())
//         } else {
//             self.epstatusclr(self.index())
//                 .write(|w| w.stallrq1().set_bit())
//         }
//     }
// }

// impl<'a> Bank<'a, OutBank> {
//     fn desc_bank(&mut self) -> &mut DeviceDescBank {
//         let idx = self.index();
//         self.desc.bank(idx, 0)
//     }

//     /// Returns true if Bank 0 is Ready and thus has data that can be read.
//     #[inline]
//     fn is_ready(&self) -> bool {
//         self.epstatus(self.index()).read().bk0rdy().bit()
//     }

//     /// Set Bank 0 Ready.
//     /// Ready means that the buffer contains data that can be read.
//     #[inline]
//     fn set_ready(&self, ready: bool) {
//         if ready {
//             self.epstatusset(self.index())
//                 .write(|w| w.bk0rdy().set_bit());
//         } else {
//             self.epstatusclr(self.index())
//                 .write(|w| w.bk0rdy().set_bit());
//         }
//     }

//     /// Acknowledges the signal that data has been received.
//     #[inline]
//     fn clear_transfer_complete(&self) {
//         // Clear bits in epintflag by writing them to 1
//         self.epintflag(self.index())
//             .write(|w| w.trcpt0().set_bit().trfail0().set_bit());
//     }

//     /// Returns true if a Received Setup interrupt has occurred.
//     /// This indicates that the read buffer holds a SETUP packet.
//     #[inline]
//     fn received_setup_interrupt(&self) -> bool {
//         self.epintflag(self.index()).read().rxstp().bit()
//     }

//     /// Acknowledges the signal that a SETUP packet was received
//     /// successfully.
//     #[inline]
//     fn clear_received_setup_interrupt(&self) {
//         // Clear bits in epintflag by writing them to 1
//         self.epintflag(self.index()).write(|w| w.rxstp().set_bit());
//     }

//     /// Writes out endpoint configuration to its in-memory descriptor.
//     fn flush_config(&mut self) {
//         let config = *self.config();
//         {
//             let desc = self.desc_bank();
//             desc.set_address(config.addr as *mut u8);
//             desc.set_endpoint_size(config.max_packet_size);
//             desc.set_multi_packet_size(0);
//             desc.set_byte_count(0);
//         }
//     }

//     /// Enables endpoint-specific interrupts.
//     fn setup_ep_interrupts(&mut self) {
//         self.epintenset(self.index())
//             .write(|w| w.rxstp().set_bit().trcpt0().set_bit());
//     }

//     /// Copies data from the bank0 buffer to the provided array. The caller
//     /// must call set_ready to indicate the buffer is free for the next
//     /// transfer.
//     pub fn read(&mut self, buf: &mut [u8]) -> UsbResult<usize> {
//         let desc = self.desc_bank();
//         let size = desc.get_byte_count() as usize;

//         if size > buf.len() {
//             return Err(UsbError::BufferOverflow);
//         }
//         unsafe {
//             desc.get_address()
//                 .copy_to_nonoverlapping(buf.as_mut_ptr(), size);
//         }

//         desc.set_byte_count(0);
//         desc.set_multi_packet_size(0);

//         Ok(size)
//     }

//     fn is_stalled(&self) -> bool {
//         self.epintflag(self.index()).read().stall0().bit()
//     }

//     fn set_stall(&mut self, stall: bool) {
//         if stall {
//             self.epstatusset(self.index())
//                 .write(|w| w.stallrq0().set_bit())
//         } else {
//             self.epstatusclr(self.index())
//                 .write(|w| w.stallrq0().set_bit())
//         }
//     }
// }

// impl<'a, T> Bank<'a, T> {
//     ep!(epcfg, EPCFG);
//     ep!(epstatusclr, EPSTATUSCLR);
//     ep!(epstatusset, EPSTATUSSET);
//     ep!(epstatus, EPSTATUS);
//     ep!(epintflag, EPINTFLAG);
//     ep!(epintenclr, EPINTENCLR);
//     ep!(epintenset, EPINTENSET);
// }

// impl Inner {
//     ep!(epcfg, EPCFG);
//     ep!(epstatus, EPSTATUS);
//     ep!(epintflag, EPINTFLAG);

//     fn bank0(&'_ self, ep: EndpointAddress) -> UsbResult<Bank<'_, OutBank>> {
//         if ep.is_in() {
//             return Err(UsbError::InvalidEndpoint);
//         }
//         let endpoints = self.endpoints.borrow();

//         if endpoints.endpoints[ep.index()].bank0.ep_type == EndpointTypeBits::Disabled {
//             return Err(UsbError::InvalidEndpoint);
//         }
//         Ok(Bank {
//             address: ep,
//             usb: self.usb(),
//             desc: self.desc.borrow_mut(),
//             endpoints,
//             _phantom: PhantomData,
//         })
//     }

//     fn bank1(&'_ self, ep: EndpointAddress) -> UsbResult<Bank<'_, InBank>> {
//         if ep.is_out() {
//             return Err(UsbError::InvalidEndpoint);
//         }
//         let endpoints = self.endpoints.borrow();

//         if endpoints.endpoints[ep.index()].bank1.ep_type == EndpointTypeBits::Disabled {
//             return Err(UsbError::InvalidEndpoint);
//         }
//         Ok(Bank {
//             address: ep,
//             usb: self.usb(),
//             desc: self.desc.borrow_mut(),
//             endpoints,
//             _phantom: PhantomData,
//         })
//     }
// }

impl Inner {
    fn open_endpoint(&self, ep_index: usize) {
        let usbhs = self.usbhs();
        match self.endpoints.ep_config[ep_index] {
            Some(ep_config) => {
                // generic configuration
                usbhs.usbhs_deveptcfg[ep_index].write(|w| {
                    match ep_config.max_packet_size {
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

                    // set end point type
                    w.eptype().bits(ep_config.ep_type as u8);

                    // force allocation
                    w.alloc().set_bit()
                });

                if ep_index == 0 {
                    rprintln!("ep {} - set ctrl", ep_index);
                    // ep0 used as ctrl endpoint
                    // Configure the Endpoint 0 configuration register
                    usbhs.usbhs_deveptcfg[0].modify(|_, w| {
                        // setup RSTDTS
                        usbhs.usbhs_deveptier_ctrl_mode()[ep_index].write(|w| w.rstdts().set_bit());
                        // setup STALLRQC
                        usbhs.usbhs_deveptidr_ctrl_mode()[ep_index]
                            .write(|w| w.stallrqc().set_bit());

                        w
                    });

                    // check that endpoint was correctly initiated
                    // Notice, re-allocation might fail in case size is larger, so be aware
                    if usbhs.usbhs_deveptisr_ctrl_mode()[ep_index]
                        .read()
                        .cfgok()
                        .bit_is_set()
                    {
                        // Endpoint configuration is successful
                        usbhs.usbhs_deveptier_ctrl_mode()[ep_index]
                            .write(|w| w.rxstpes().set_bit());
                        // Enable Endpoint Interrupt
                        self.enable_endpoint_interrupt(ep_index);
                    } else {
                        todo!();
                        // rprintln!("ep {} - set {:?}", ep_index, ep_config.ep_type);
                        // usbhs.usbhs_deveptcfg[ep_index].modify(|_, w| {
                        //     // direction,
                        //     // 0 (OUT): The endpoint direction is OUT.
                        //     // 1 (IN): The endpoint direction is IN (nor for control endpoints).
                        //     w.epdir().bit(ep_config.ep_dir == UsbDirection::In);

                        //     // autosw, Per: do we really need this if not supporting multiple banks
                        //     w.autosw().set_bit();

                        //     // set nbtrans
                        //     if ep_config.ep_type == EndpointType::Isochronous {
                        //         w.nbtrans()._1_trans();
                        //         todo!()
                        //     }

                        //     w
                        // });
                    };
                }
            }
            None => {}
        }
    }

    fn enable(&mut self) {
        // Per: Comments
        // - x5 sets device descriptors in HW
        // - x5 configures end points before attach

        rprintln!("inner:enable");

        let usbhs = self.usbhs();
        // 39.5.2
        usbhs.usbhs_ctrl.write(|w| {
            w.usbe().set_bit(); // enable usb_hs
            w.uimod().set_bit(); // enable device mode
            w.vbushwc().set_bit() // must be set?
        });

        //  wait until clock stable
        while usbhs.usbhs_sr.read().clkusable().bit_is_clear() {}

        // normal mode, both fs and hs available, will autodetect
        usbhs.usbhs_devctrl.modify(|_, w| w.spdconf().normal());

        // enable interrupts
        usbhs.usbhs_devier.write(|w| {
            w.eorstes().set_bit();
            w.suspes().set_bit();
            w.wakeupes().set_bit();
            w.sofes().set_bit() // should we use this?
        });

        // Manually set the Suspend Interrupt
        usbhs.usbhs_devifr.write(|w| w.susps().set_bit());

        // Clear/Ack interrupts
        usbhs.usbhs_devicr.write(|w| {
            w.eorstc().set_bit();
            w.sofc().set_bit();
            w.wakeupc().set_bit()
        });

        // attach the device
        usbhs.usbhs_devctrl.modify(|_, w| w.detach().clear_bit());

        // setup endpoints
        for ep_index in 0..MAX_ENDPOINTS {
            self.open_endpoint(ep_index);
        }

        // un-freeze the clock, we want it enabled at all times
        usbhs.usbhs_ctrl.modify(|_, w| w.frzclk().clear_bit());
    }

    // 39.6.2.3 USB Reset
    // The USB bus reset is managed by hardware. It is initiated by a connected host.
    // When a USB reset is detected on the USB line, the following operations are performed by the controller:
    // • All endpoints are disabled, except the default control endpoint.
    // • The default control endpoint is reset (see 39.6.2.4. Endpoint Reset for more details).
    // • The data toggle sequence of the default control endpoint is cleared.
    // • At the end of the reset process, the End of Reset (USBHS_DEVISR.EORST) bit is set.
    // • During a reset, the USBHS automatically switches to High-speed mode if the host is High-speed-capable (the
    //   reset is called High-speed reset). The user should observe the USBHS_SR.SPEED field to know the speed
    //   running at the end of the reset (USBHS_DEVISR.EORST = 1).

    // 39.6.2.4 Endpoint Reset
    // An endpoint can be reset at any time by writing a one to the Endpoint x Reset bit USBHS_DEVEPT.EPRSTx. This
    // is recommended before using an endpoint upon hardware reset or when a USB bus reset has been received. This
    // resets:
    // • The internal state machine of the endpoint.
    // • The receive and transmit bank FIFO counters,
    // • All registers of this endpoint (USBHS_DEVEPTCFGx, USBHS_DEVEPTISRx, the Endpoint x
    //   Control (USBHS_DEVEPTIMRx) register), except its configuration (USBHS_DEVEPTCFGx.ALLOC,
    //   USBHS_DEVEPTCFGx.EPBK, USBHS_DEVEPTCFGx.EPSIZE, USBHS_DEVEPTCFGx.EPDIR,
    //   USBHS_DEVEPTCFGx.EPTYPE) and the Data Toggle Sequence (USBHS_DEVEPTISRx.DTSEQ) field.
    //
    // Note: The interrupt sources located in USBHS_DEVEPTISRx are not cleared when a USB bus reset has been
    // received.
    // The endpoint configuration remains active and the endpoint is still enabled.

    // Called when a USB Reset is detected
    fn reset(&self) {
        rprintln!("inner:reset");
        let usbhs = self.usbhs();
        // assume USB endpoints already configured by `enable`
        // we don't need to reset ep0, done by HW
        usbhs.usbhs_devept.modify(|_, w| {
            // set the reset bit(s)
            w.eprst1().set_bit();
            w.eprst2().set_bit();
            w.eprst3().set_bit();
            w.eprst4().set_bit();
            w.eprst5().set_bit();
            w.eprst6().set_bit();
            w.eprst7().set_bit();
            w.eprst8().set_bit();
            w.eprst9().set_bit()
        });

        usbhs.usbhs_devept.modify(|_, w| {
            // clear the reset bit(s)
            w.eprst1().clear_bit();
            w.eprst2().clear_bit();
            w.eprst3().clear_bit();
            w.eprst4().clear_bit();
            w.eprst5().clear_bit();
            w.eprst6().clear_bit();
            w.eprst7().clear_bit();
            w.eprst8().clear_bit();
            w.eprst9().clear_bit()
        });

        self.open_endpoint(0);
    }
    //     /// Enables/disables the Start Of Frame (SOF) interrupt
    //     fn sof_interrupt(&self, enable: bool) {
    //         if enable {
    //             self.usb().intenset.write(|w| w.sof().set_bit());
    //         } else {
    //             self.usb().intenclr.write(|w| w.sof().set_bit());
    //         }
    //     }
    //     /// protocol_reset is called by the USB HAL when it detects the host has
    //     /// performed a USB reset.
    //     fn protocol_reset(&self) {
    //         self.flush_eps(FlushConfigMode::ProtocolReset);
    //     }

    //     fn suspend(&self) {}
    //     fn resume(&self) {}

    fn alloc_ep(
        &mut self,
        dir: UsbDirection,
        addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> UsbResult<EndpointAddress> {
        rprintln!(
            "inner:alloc_ep dir:{:?}, addr:{:?}, type:{:?}, size:{}",
            dir,
            addr,
            ep_type,
            max_packet_size
        );
        let idx = match addr {
            None => self.endpoints.find_free_endpoint()?,
            Some(addr) => addr.index(),
        };

        let addr =
            self.endpoints
                .allocate_endpoint(dir, idx, ep_type, max_packet_size, interval)?;

        Ok(addr)
    }

    fn set_device_address(&self, addr: u8) {
        todo!("inner:set_device_address {}", addr);
    }

    //     fn check_sof_interrupt(&self) -> bool {
    //         if self.usb().intflag.read().sof().bit() {
    //             self.usb().intflag.write(|w| w.sof().set_bit());
    //             return true;
    //         }
    //         false
    //     }

    fn poll(&self) -> PollResult {
        rprintln!("inner:poll");

        // Safety: Usbd owns the USBHS
        let usbhs = self.usbhs();
        let dev_ctrl = usbhs.usbhs_devctrl.read().bits();
        rprintln!("dev_ctrl {:#10x}", dev_ctrl);
        let ctrl = usbhs.usbhs_ctrl.read().bits();
        rprintln!("ctrl {:x}", ctrl);
        let dev_isr = usbhs.usbhs_devisr.read();
        rprintln!("dev_irs : {:#010x}", dev_isr.bits());

        if dev_isr.eorst().bit_is_set() {
            // EORST - End of Reset

            rprintln!("eorst");
            let speed = usbhs.usbhs_sr.read().speed();
            rprintln!(
                "speed {:?}",
                match speed.bits() {
                    0 => "full speed",
                    1 => "high speed",
                    2 => "low speed",
                    _ => "reserved",
                }
            );

            // clear the eorst interrupt
            usbhs.usbhs_devicr.write(|w| w.eorstc().set_bit());

            return PollResult::Reset;
        }

        // As the suspend & wakup interrupts/states cannot distinguish between
        // unconnected & unsuspended, we do not handle them to avoid spurious transitions.

        let mut ep_out = 0;
        let mut ep_in_complete = 0;
        let mut ep_setup = 0;

        // for now just care about ep0
        // a bit ugly
        if dev_isr.pep_0().bit_is_set() {
            rprintln!("pep0");

            // uint16_t count = (USB_REG->DEVEPTISR[ep_ix] &
            //     DEVEPTISR_BYCT) >> DEVEPTISR_BYCT_Pos;

            let sr = usbhs.usbhs_deveptisr_ctrl_mode()[0].read();

            // setup packet?
            if sr.rxstpi().bit_is_set() {
                rprintln!("rxstpi");
                // setup packet received
                let sr = usbhs.usbhs_deveptisr_ctrl_mode()[0].read();
                let count = sr.byct().bits() as usize;
                rprintln!("count {}", count);

                // Disable RXSTPI interrupt?
                // usb_hs.usbhs_deveptidr_ctrl_mode()[0].write(|w| w.rxstpec().set_bit());

                return PollResult::Data {
                    ep_out: 0,
                    ep_in_complete: 0,
                    ep_setup: 1, // setup occurred at endpoint 0
                };
            };

            // out packet done
            if sr.rxouti().bit_is_set() {
                rprintln!("rxouti");
                // could be that we should read the out packet to confirm
                panic!();
                return PollResult::Data {
                    ep_out: 1, // fifo ready to receive new data
                    ep_in_complete: 0,
                    ep_setup: 0,
                };
            };

            // in packet done
            if sr.txini().bit_is_set() {
                rprintln!("txini");
                todo!();
                return PollResult::Data {
                    ep_out: 0,
                    ep_in_complete: 1, // data sent on ep0
                    ep_setup: 0,
                };
            };

            panic!("should receive setup packet")
        }
        PollResult::None

        //         let intbits = self.usb().epintsmry.read().bits();

        //         for ep in 0..8u16 {
        //             let mask = 1 << ep;

        //             let idx = ep as usize;

        //             if (intbits & mask) != 0 {
        //                 if let Ok(bank1) = self.bank1(EndpointAddress::from_parts(idx, UsbDirection::In)) {
        //                     if bank1.is_transfer_complete() {
        //                         bank1.clear_transfer_complete();
        //                         ep_in_complete |= mask;
        //                     }
        //                 }
        //             }

        //             // Can't test intbits, because bk0rdy doesn't interrupt
        //             if let Ok(bank0) = self.bank0(EndpointAddress::from_parts(idx, UsbDirection::Out)) {
        //                 if bank0.received_setup_interrupt() {
        //                     ep_setup |= mask;

        //                     // The RXSTP interrupt is not cleared here, because doing so
        //                     // would allow the USB hardware to overwrite the received
        //                     // data, potentially before it is `read()` - see SAMD5x
        //                     // datasheet "38.6.2.6 Management of SETUP Transactions".
        //                     // Setup events are only relevant for control endpoints, and
        //                     // in typical USB devices, endpoint 0 is the only control
        //                     // endpoint. The usb-device `poll()` method, which calls
        //                     // this `poll()`, will immediately `read()` endpoint 0 when
        //                     // its setup bit is set.
        //                 }

        //                 // Clear the transfer complete and transfer failed interrupt flags
        //                 // so that execution leaves the USB interrupt until the host makes
        //                 // another transaction.  The transfer failed flag may have been set
        //                 // if an OUT transaction wasn't read() from the endpoint by the
        //                 // Class; the hardware will have NAKed (unless the endpoint is
        //                 // isochronous) and the host may retry.
        //                 bank0.clear_transfer_complete();

        //                 // Use the bk0rdy flag via is_ready() to indicate that data has been
        //                 // received successfully, rather than the interrupting trcpt0 via
        //                 // is_transfer_ready(), because data may have been received on an
        //                 // earlier poll() which cleared trcpt0.  bk0rdy is cleared in the
        //                 // endpoint read().
        //                 if bank0.is_ready() {
        //                     ep_out |= mask;
        //                 }
        //             }
        //         }

        //         if ep_out == 0 && ep_in_complete == 0 && ep_setup == 0 {
        //             PollResult::None
        //         } else {
        //             PollResult::Data {
        //                 ep_out,
        //                 ep_in_complete,
        //                 ep_setup,
        //             }
        //         }
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        rprintln!("inner:write");
        rprintln!("ep_addr {:?}", ep_addr);
        rprintln!("buf {:02x?}", buf);
        rprintln!("buf.len {:?}", buf.len());

        let ep_index = ep_addr.index();
        rprintln!("ep_index {}", ep_index);
        self.write_fifo(ep_index, buf);

        let usbhs = self.usbhs();

        // clear TXINI to send the package
        usbhs.usbhs_devepticr_ctrl_mode()[0].write(|w| w.txinic().set_bit());
        // enable TXINI interrupt
        usbhs.usbhs_deveptier_ctrl_mode()[0].write(|w| w.txines().set_bit());
        // enable RXOUTI interrupt
        usbhs.usbhs_deveptier_ctrl_mode()[0].write(|w| w.rxoutes().set_bit());

        Ok(buf.len())

        //         let mut bank = self.bank1(ep)?;

        //         if bank.is_ready() {
        //             // Waiting for the host to pick up the existing data
        //             return Err(UsbError::WouldBlock);
        //         }

        //         let size = bank.write(buf);

        //         bank.clear_transfer_complete();
        //         bank.set_ready(true); // ready to be sent

        //         size
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        rprintln!("inner:read");
        rprintln!("ep_addr {:?}", ep_addr);
        rprintln!("buf.len {:?}", buf.len());

        let ep_index = ep_addr.index();

        let usbhs = self.usbhs();

        let sr = usbhs.usbhs_deveptisr_ctrl_mode()[0].read();
        let count = sr.byct().bits() as usize;
        rprintln!("--- read count {}", count);

        self.read_fifo(ep_index, &mut buf[0..count]);

        rprintln!("--- read buf {:x?}", &buf[0..count]);

        // Clear RXSTPI interrupt, and make FIFO available
        usbhs.usbhs_devepticr_ctrl_mode()[0].write(|w| w.rxstpic().set_bit());

        Ok(count)

        //         let mut bank = self.bank0(ep)?;
        //         let rxstp = bank.received_setup_interrupt();

        //         if bank.is_ready() || rxstp {
        //             let size = bank.read(buf);

        //             if rxstp {
        //                 bank.clear_received_setup_interrupt();
        //             }

        //             bank.clear_transfer_complete();
        //             bank.set_ready(false);

        //             size
        //         } else {
        //             Err(UsbError::WouldBlock)
        //         }
    }

    fn is_stalled(&self, ep: EndpointAddress) -> bool {
        rprintln!("inner:is_stalled");
        todo!()
    }

    fn set_stalled(&self, ep: EndpointAddress, stalled: bool) {
        rprintln!("inner:set_stalled, {}", stalled);
        // self.set_stall(ep, stalled);
    }
}

// impl UsbBus {
//     /// Enables the Start Of Frame (SOF) interrupt
//     pub fn enable_sof_interrupt(&self) {
//         disable_interrupts(|cs| self.inner.borrow(cs).borrow_mut().sof_interrupt(true))
//     }

//     /// Disables the Start Of Frame (SOF) interrupt
//     pub fn disable_sof_interrupt(&self) {
//         disable_interrupts(|cs| self.inner.borrow(cs).borrow_mut().sof_interrupt(false))
//     }

//     /// Checks, and clears if set, the Start Of Frame (SOF) interrupt flag
//     pub fn check_sof_interrupt(&self) -> bool {
//         disable_interrupts(|cs| self.inner.borrow(cs).borrow_mut().check_sof_interrupt())
//     }
// }

impl usb_device::bus::UsbBus for UsbBus {
    fn enable(&mut self) {
        interrupt::free(|cs| unsafe { &mut *self.inner.borrow(cs).get() }.enable());
    }

    fn reset(&self) {
        interrupt::free(|cs| unsafe { &mut *self.inner.borrow(cs).get() }.reset());
    }

    fn suspend(&self) {
        todo!()
        // disable_interrupts(|cs| self.inner.borrow(cs).borrow().suspend())
    }

    fn resume(&self) {
        todo!()
        // disable_interrupts(|cs| self.inner.borrow(cs).borrow().resume())
    }

    fn alloc_ep(
        &mut self,
        dir: UsbDirection,
        addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> UsbResult<EndpointAddress> {
        interrupt::free(|cs| {
            unsafe { &mut *self.inner.borrow(cs).get() }.alloc_ep(
                dir,
                addr,
                ep_type,
                max_packet_size,
                interval,
            )
        })
    }

    fn set_device_address(&self, addr: u8) {
        todo!();
        // disable_interrupts(|cs| self.inner.borrow(cs).borrow().set_device_address(addr))
    }

    fn poll(&self) -> PollResult {
        interrupt::free(|cs| unsafe { &mut *self.inner.borrow(cs).get() }.poll())
    }

    fn write(&self, ep: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        interrupt::free(|cs| unsafe { &mut *self.inner.borrow(cs).get() }.write(ep, buf))
    }

    fn read(&self, ep: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        interrupt::free(|cs| unsafe { &mut *self.inner.borrow(cs).get() }.read(ep, buf))
    }

    fn set_stalled(&self, ep: EndpointAddress, stalled: bool) {
        interrupt::free(|cs| unsafe { &mut *self.inner.borrow(cs).get() }.set_stalled(ep, stalled));
    }

    fn is_stalled(&self, ep: EndpointAddress) -> bool {
        interrupt::free(|cs| unsafe { &mut *self.inner.borrow(cs).get() }.is_stalled(ep))
    }
}
