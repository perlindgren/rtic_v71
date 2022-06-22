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
use usb_device::bus::PollResult;
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result as UsbResult, UsbDirection, UsbError};

use rtt_target::rprintln;

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
    endpoints: [Option<EPConfig>; 10],
}

impl Endpoints {
    fn new() -> Self {
        Self {
            endpoints: [None; MAX_ENDPOINTS],
        }
    }

    fn find_free_endpoint(&self) -> UsbResult<usize> {
        // start with 1 because 0 is reserved for Control
        for idx in 1..MAX_ENDPOINTS {
            if self.endpoints[idx] == None {
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
        if self.endpoints[idx] != None {
            return Err(UsbError::EndpointOverflow);
        }

        self.endpoints[idx] = Some(EPConfig::new(ep_type, max_packet_size));

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
    fn usbhs(&self) -> &pac::usbhs::RegisterBlock {
        // Safety: Usbd owns the USBHS
        unsafe { &*pac::USBHS::ptr() }
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

        // un-freeze the clock, we want it enabled at all times
        usbhs.usbhs_ctrl.modify(|_, w| w.frzclk().clear_bit());
    }

    //     /// Enables/disables the Start Of Frame (SOF) interrupt
    //     fn sof_interrupt(&self, enable: bool) {
    //         if enable {
    //             self.usb().intenset.write(|w| w.sof().set_bit());
    //         } else {
    //             self.usb().intenclr.write(|w| w.sof().set_bit());
    //         }
    //     }

    //     /// Configures all endpoints based on prior calls to alloc_ep().
    //     fn flush_eps(&self, mode: FlushConfigMode) {
    //         for idx in 0..8 {
    //             match (mode, idx) {
    //                 // A flush due to a protocol reset need not reconfigure endpoint 0,
    //                 // except for enabling its interrupts.
    //                 (FlushConfigMode::ProtocolReset, 0) => {
    //                     self.setup_ep_interrupts(EndpointAddress::from_parts(idx, UsbDirection::Out));
    //                     self.setup_ep_interrupts(EndpointAddress::from_parts(idx, UsbDirection::In));
    //                 }
    //                 // A full flush configures all provisioned endpoints + enables interrupts.
    //                 // Endpoints 1-8 have identical behaviour when flushed due to protocol reset.
    //                 (FlushConfigMode::Full, _) | (FlushConfigMode::ProtocolReset, _) => {
    //                     // Write bank configuration & endpoint type.
    //                     self.flush_ep(idx);
    //                     // Endpoint interrupts are configured after the write to EPTYPE, as it appears
    //                     // writes to EPINTEN*[n] do not take effect unless the
    //                     // endpoint is already somewhat configured. The datasheet is
    //                     // ambiguous here, section 38.8.3.7 (Device Interrupt EndPoint Set n)
    //                     // of the SAM D5x/E5x states:
    //                     //    "This register is cleared by USB reset or when EPEN[n] is zero"
    //                     // EPEN[n] is not a register that exists, nor does it align with any other
    //                     // terminology. We assume this means setting EPCFG[n] to a
    //                     // non-zero value, but we do interrupt configuration last to
    //                     // be sure.
    //                     self.setup_ep_interrupts(EndpointAddress::from_parts(idx, UsbDirection::Out));
    //                     self.setup_ep_interrupts(EndpointAddress::from_parts(idx, UsbDirection::In));
    //                 }
    //             }
    //         }
    //     }

    //     /// flush_ep commits bank descriptor information for the endpoint pair,
    //     /// and enables the endpoint according to its type.
    //     fn flush_ep(&self, idx: usize) {
    //         let cfg = self.epcfg(idx);
    //         let info = &self.endpoints.borrow().endpoints[idx];
    //         // Write bank descriptors first. We do this so there is no period in
    //         // which the endpoint is enabled but has an invalid descriptor.
    //         if let Ok(mut bank) = self.bank0(EndpointAddress::from_parts(idx, UsbDirection::Out)) {
    //             bank.flush_config();
    //         }
    //         if let Ok(mut bank) = self.bank1(EndpointAddress::from_parts(idx, UsbDirection::In)) {
    //             bank.flush_config();
    //         }

    //         // Set the endpoint type. At this point, the endpoint is enabled.
    //         cfg.modify(|_, w| unsafe {
    //             w.eptype0()
    //                 .bits(info.bank0.ep_type as u8)
    //                 .eptype1()
    //                 .bits(info.bank1.ep_type as u8)
    //         });
    //     }

    //     /// setup_ep_interrupts enables interrupts for the given endpoint address.
    //     fn setup_ep_interrupts(&self, ep_addr: EndpointAddress) {
    //         if ep_addr.is_out() {
    //             if let Ok(mut bank) = self.bank0(ep_addr) {
    //                 bank.setup_ep_interrupts();
    //             }
    //         } else if let Ok(mut bank) = self.bank1(ep_addr) {
    //             bank.setup_ep_interrupts();
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
        rprintln!("inner:alloc_ep");
        //         let intflags = self.usb().intflag.read();
        //         if intflags.eorst().bit() {
        //             // end of reset interrupt
        //             self.usb().intflag.write(|w| w.eorst().set_bit());
        //             return PollResult::Reset;
        //         }
        //         // As the suspend & wakup interrupts/states cannot distinguish between
        //         // unconnected & unsuspended, we do not handle them to avoid spurious
        //         // transitions.

        //         let mut ep_out = 0;
        //         let mut ep_in_complete = 0;
        //         let mut ep_setup = 0;

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
        PollResult::None
    }

    fn write(&self, ep: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        rprintln!("inner:write");
        todo!()

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

    fn read(&self, ep: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        rprintln!("inner:read");
        todo!()

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
        rprintln!("inner:set_stalled");
        todo!()
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
        todo!()
        // disable_interrupts(|cs| self.inner.borrow(cs).borrow().protocol_reset())
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
