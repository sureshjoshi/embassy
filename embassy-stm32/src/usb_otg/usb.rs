use core::marker::PhantomData;
use core::task::Poll;

use embassy_cortex_m::interrupt::InterruptExt;
use embassy_hal_common::{into_ref, Peripheral};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    self, Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event, Unsupported,
};
use futures::future::poll_fn;

use super::*;
use crate::gpio::sealed::AFType;
use crate::pac::otgfs::{regs, vals};
use crate::rcc::sealed::RccPeripheral;

// const EP_COUNT: usize = 6; // TODO unhardcode

// const NEW_AW: AtomicWaker = AtomicWaker::new();
// static BUS_WAKER: AtomicWaker = AtomicWaker::new();
// static EP_IN_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];
// static EP_OUT_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];

macro_rules! config_ulpi_pins {
    ($($pin:ident),*) => {
        into_ref!($($pin),*);
        // NOTE(unsafe) Exclusive access to the registers
        critical_section::with(|_| unsafe {
            $(
                $pin.set_as_af($pin.af_num(), AFType::OutputPushPull);
                #[cfg(gpio_v2)]
                $pin.set_speed(crate::gpio::Speed::VeryHigh);
            )*
        })
    };
}

// From `synopsys-usb-otg` crate:
// This calculation doesn't correspond to one in a Reference Manual.
// In fact, the required number of words is higher than indicated in RM.
// The following numbers are pessimistic and were figured out empirically.
const RX_FIFO_EXTRA_SIZE_WORDS: u16 = 30;

/// USB PHY type
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PhyType {
    /// Internal Full-Speed PHY
    ///
    /// Available on most High-Speed peripherals.
    InternalFullSpeed,
    /// Internal High-Speed PHY
    ///
    /// Available on a few STM32 chips.
    InternalHighSpeed,
    /// External ULPI High-Speed PHY
    ExternalHighSpeed,
}

pub struct State<const EP_COUNT: usize> {
    ep_in_wakers: [AtomicWaker; EP_COUNT],
    ep_out_wakers: [AtomicWaker; EP_COUNT],
    bus_waker: AtomicWaker,
}

impl<const EP_COUNT: usize> State<EP_COUNT> {
    pub const fn new() -> Self {
        const NEW_AW: AtomicWaker = AtomicWaker::new();

        Self {
            ep_in_wakers: [NEW_AW; EP_COUNT],
            ep_out_wakers: [NEW_AW; EP_COUNT],
            bus_waker: NEW_AW,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct EndpointData {
    ep_type: EndpointType,
    size_words: u16,
}

pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    ep_in: [Option<EndpointData>; MAX_EP_COUNT],
    ep_out: [Option<EndpointData>; MAX_EP_COUNT],
    _phy_type: PhyType,
}

impl<'d, T: Instance> Driver<'d, T> {
    /// Initializes USB OTG peripheral with internal Full-Speed PHY
    pub fn new_fs(
        _peri: impl Peripheral<P = T> + 'd,
        irq: impl Peripheral<P = T::Interrupt> + 'd,
        dp: impl Peripheral<P = impl DpPin<T>> + 'd,
        dm: impl Peripheral<P = impl DmPin<T>> + 'd,
    ) -> Self {
        into_ref!(dp, dm, irq);

        irq.set_handler(Self::on_interrupt);
        irq.unpend();
        irq.enable();

        unsafe {
            dp.set_as_af(dp.af_num(), AFType::OutputPushPull);
            dm.set_as_af(dm.af_num(), AFType::OutputPushPull);

            #[cfg(stm32l4)]
            {
                crate::peripherals::PWR::enable();
                pac::PWR.cr2().modify(|w| w.set_usv(true));
            }

            <T as RccPeripheral>::enable();
            <T as RccPeripheral>::reset();

            let r = T::regs();
            let core_id = r.cid().read().0;
            info!("Core id {:08x}", core_id);

            // Wait for AHB ready.
            while !r.grstctl().read().ahbidl() {}

            // Configure as device.
            r.gusbcfg().write(|w| {
                w.set_fdmod(true); // Force device mode
                w.set_physel(true); // internal FS PHY
            });

            // Soft-reset
            while !r.grstctl().read().ahbidl() {}
            r.grstctl().write(|w| w.set_csrst(true));
            while r.grstctl().read().csrst() {}

            // Enable internal USB transceiver
            r.gccfg().modify(|w| {
                w.set_pwrdwn(true);
            });

            // Configuring Vbus sense and SOF output
            match core_id {
                0x0000_1200 | 0x0000_1100 => {
                    // F429-like chips have the GCCFG.NOVBUSSENS bit
                    r.gccfg().modify(|w| {
                        w.set_novbussens(true);
                        w.set_vbusasen(false);
                        w.set_vbusbsen(false);
                        w.set_sofouten(false);
                    });
                }
                0x0000_2000 | 0x0000_2100 | 0x0000_2300 | 0x0000_3000 | 0x0000_3100 => {
                    // F446-like chips have the GCCFG.VBDEN bit with the opposite meaning
                    r.gccfg().modify(|w| {
                        w.set_vbden(false);
                    });

                    // Force B-peripheral session
                    r.gotgctl().modify(|w| {
                        w.set_bvaloen(true);
                        w.set_bvaloval(true);
                    });
                }
                _ => defmt::unimplemented!("Unknown USB core id {:X}", core_id),
            }

            // Enable PHY clock
            r.pcgcctl().write(|w| {
                w.set_stppclk(false);
            });

            // Soft disconnect.
            r.dctl().write(|w| w.set_sdis(true));

            // Set speed.
            r.dcfg().write(|w| {
                w.set_pfivl(vals::Pfivl::FRAME_INTERVAL_80);
                w.set_dspd(vals::Dspd::FULL_SPEED_INTERNAL);
            });

            // Unmask transfer complete EP interrupt
            r.diepmsk().write(|w| {
                w.set_xfrcm(true);
            });

            // Unmask and clear core interrupts
            Bus::<T>::restore_irqs();
            r.gintsts().write_value(regs::Gintsts(0xFFFF_FFFF));

            // Unmask global interrupt
            r.gahbcfg().write(|w| {
                w.set_gint(true); // unmask global interrupt
            });

            // Connect
            r.dctl().write(|w| w.set_sdis(false));
        }

        Self {
            phantom: PhantomData,
            ep_in: [None; MAX_EP_COUNT],
            ep_out: [None; MAX_EP_COUNT],
            _phy_type: PhyType::InternalFullSpeed,
        }
    }

    /// Initializes USB OTG peripheral with external High-Speed PHY
    pub fn new_hs_ulpi(
        _peri: impl Peripheral<P = T> + 'd,
        ulpi_clk: impl Peripheral<P = impl UlpiClkPin<T>> + 'd,
        ulpi_dir: impl Peripheral<P = impl UlpiDirPin<T>> + 'd,
        ulpi_nxt: impl Peripheral<P = impl UlpiNxtPin<T>> + 'd,
        ulpi_stp: impl Peripheral<P = impl UlpiStpPin<T>> + 'd,
        ulpi_d0: impl Peripheral<P = impl UlpiD0Pin<T>> + 'd,
        ulpi_d1: impl Peripheral<P = impl UlpiD1Pin<T>> + 'd,
        ulpi_d2: impl Peripheral<P = impl UlpiD2Pin<T>> + 'd,
        ulpi_d3: impl Peripheral<P = impl UlpiD3Pin<T>> + 'd,
        ulpi_d4: impl Peripheral<P = impl UlpiD4Pin<T>> + 'd,
        ulpi_d5: impl Peripheral<P = impl UlpiD5Pin<T>> + 'd,
        ulpi_d6: impl Peripheral<P = impl UlpiD6Pin<T>> + 'd,
        ulpi_d7: impl Peripheral<P = impl UlpiD7Pin<T>> + 'd,
    ) -> Self {
        config_ulpi_pins!(
            ulpi_clk, ulpi_dir, ulpi_nxt, ulpi_stp, ulpi_d0, ulpi_d1, ulpi_d2, ulpi_d3, ulpi_d4, ulpi_d5, ulpi_d6,
            ulpi_d7
        );

        Self {
            phantom: PhantomData,
            ep_in: [None; MAX_EP_COUNT],
            ep_out: [None; MAX_EP_COUNT],
            _phy_type: PhyType::ExternalHighSpeed,
        }
    }

    fn on_interrupt(_: *mut ()) {
        unsafe {
            trace!("USB IRQ");
            let r = T::regs();
            let ints = r.gintsts().read();
            if ints.wkupint() || ints.usbsusp() || ints.usbrst() || ints.enumdne() {
                r.gintmsk().write(|_| {});
                T::state().bus_waker.wake();
            }

            if ints.rxflvl() {
                r.gintmsk().modify(|w| {});
                let ep_index = r.grxstsr().read().epnum() as usize;
                assert!(ep_index <= T::ENDPOINT_COUNT);
                T::state().ep_out_wakers[ep_index].wake();
            }
        }
    }

    fn rx_fifo_size_words(&self) -> u16 {
        // USB out direction (host perspective) is RX direction (device perspective)
        self.ep_out
            .iter()
            .map(|ep| ep.map(|ep| ep.size_words).unwrap_or(0))
            .sum()
    }

    fn tx_fifo_size_words(&self) -> u16 {
        // USB in direction (host perspective) is TX direction (device perspective)
        self.ep_in
            .iter()
            .map(|ep| ep.map(|ep| ep.size_words).unwrap_or(0))
            .sum()
    }

    // Returns total amount of words (u32) allocated in dedicated FIFO
    fn allocated_fifo_words(&self) -> u16 {
        RX_FIFO_EXTRA_SIZE_WORDS + self.rx_fifo_size_words() + self.tx_fifo_size_words()
    }

    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<Endpoint<'d, T, D>, EndpointAllocError> {
        trace!(
            "allocating type={:?} mps={:?} interval={}, dir={:?}",
            ep_type,
            max_packet_size,
            interval,
            D::dir()
        );

        let size_words = (max_packet_size + 3) / 4;
        if size_words + self.allocated_fifo_words() > T::FIFO_DEPTH_WORDS {
            error!("Not enough FIFO capacity");
            return Err(EndpointAllocError);
        }

        let eps = if D::dir() == Direction::In {
            &mut self.ep_in
        } else {
            &mut self.ep_out
        };

        // Find free endpoint slot
        let slot = eps.iter_mut().enumerate().find(|(i, ep)| {
            if *i == 0 && ep_type != EndpointType::Control {
                // reserved for control pipe
                false
            } else {
                ep.is_none()
            }
        });

        let index = match slot {
            Some((index, ep)) => {
                *ep = Some(EndpointData { ep_type, size_words });
                index
            }
            None => {
                error!("No free endpoints available");
                return Err(EndpointAllocError);
            }
        };

        trace!("  index={}", index);

        // EP0 has special MPSIZ values
        let mpsiz: u16 = if index == 0 {
            match max_packet_size {
                8 => 0b11 as u16,
                16 => 0b10 as u16,
                32 => 0b01 as u16,
                64 => 0b00 as u16,
                other => panic!("Unsupported EP0 size: {}", other),
            }
        } else {
            max_packet_size
        };

        let r = T::regs();
        match D::dir() {
            Direction::Out => {
                if index == 0 {
                    unsafe {
                        r.doepctl(index).write(|w| {
                            w.set_cnak(true);
                            w.set_mpsiz(mpsiz);
                            w.set_epena(true);
                        });
                        r.doeptsiz(index).write(|w| {
                            w.set_rxdpid_stupcnt(1);
                            w.set_pktcnt(1);
                            w.set_xfrsiz(max_packet_size as _);
                        });
                    }
                } else {
                    unsafe {
                        r.doepctl(index).write(|w| {
                            w.set_sd0pid_sevnfrm(true);
                            w.set_cnak(true);
                            w.set_epena(true);
                            w.set_usbaep(true);
                            w.set_eptyp(to_eptyp(ep_type));
                            w.set_mpsiz(mpsiz);
                        });
                    }
                }
            }
            Direction::In => {
                if index == 0 {
                    unsafe {
                        r.diepctl(index).write(|w| {
                            w.set_snak(true);
                            w.set_mpsiz(mpsiz);
                        });
                        r.dieptsiz(index).write(|w| {
                            w.set_pktcnt(0);
                            w.set_xfrsiz(max_packet_size as _);
                        });
                    }
                } else {
                    unsafe {
                        r.diepctl(index).write(|w| {
                            w.set_snak(true);
                            w.set_usbaep(true);
                            w.set_eptyp(to_eptyp(ep_type));
                            w.set_sd0pid_sevnfrm(true);
                            w.set_txfnum(index as _);
                            w.set_mpsiz(mpsiz);
                        });
                        // DIEPTSIZx is set during transfer
                    }
                }
            }
        }

        Ok(Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, D::dir()),
                ep_type,
                max_packet_size,
                interval,
            },
        })
    }
}

impl<'d, T: Instance> embassy_usb_driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, Out>;
    type EndpointIn = Endpoint<'d, T, In>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(ep_type, max_packet_size, interval)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(ep_type, max_packet_size, interval)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let ep_out = self
            .alloc_endpoint(EndpointType::Control, control_max_packet_size, 0)
            .unwrap();
        let ep_in = self
            .alloc_endpoint(EndpointType::Control, control_max_packet_size, 0)
            .unwrap();
        assert_eq!(ep_out.info.addr.index(), 0);
        assert_eq!(ep_in.info.addr.index(), 0);

        let r = T::regs();

        unsafe {
            // Configure RX fifo size. All endpoints share the same FIFO area.
            let rx_fifo_size_words = RX_FIFO_EXTRA_SIZE_WORDS + self.rx_fifo_size_words();
            r.grxfsiz().modify(|w| w.set_rxfd(rx_fifo_size_words));
            trace!("configuring rx fifo size={}", rx_fifo_size_words);

            // Configure TX (USB in direction) fifo size for each endpoint
            let mut fifo_top = rx_fifo_size_words;
            for i in 0..T::ENDPOINT_COUNT {
                let ep_size_words = self.ep_in[i].map(|ep| ep.size_words).unwrap_or(0);
                trace!(
                    "configuring tx fifo ep={}, offset={}, size={}",
                    i,
                    fifo_top,
                    ep_size_words
                );

                let dieptxf = if i == 0 { r.dieptxf0() } else { r.dieptxf(i - 1) };
                dieptxf.write(|w| {
                    w.set_fd(ep_size_words);
                    w.set_sa(fifo_top);
                });

                fifo_top += ep_size_words;
            }

            assert!(
                fifo_top <= T::FIFO_DEPTH_WORDS,
                "FIFO allocations exceeded maximum capacity"
            );

            for ep in self.ep_in {
                if let Some(ep) = ep {}
            }
        }

        trace!("enabled");

        (
            Bus { phantom: PhantomData },
            ControlPipe {
                _phantom: PhantomData,
                max_packet_size: control_max_packet_size,
                ep_out,
                ep_in,
            },
        )
    }
}

pub struct Bus<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Bus<'d, T> {
    fn restore_irqs() {
        let r = T::regs();
        unsafe {
            r.gintmsk().write(|w| {
                w.set_usbrst(true);
                w.set_enumdnem(true);
                w.set_usbsuspm(true);
                w.set_wuim(true);
                w.set_iepint(true);
                w.set_rxflvlm(true);
            });
        }
    }
}

impl<'d, T: Instance> embassy_usb_driver::Bus for Bus<'d, T> {
    async fn poll(&mut self) -> Event {
        poll_fn(move |cx| unsafe {
            let r = T::regs();

            T::state().bus_waker.register(cx.waker());

            let ints = r.gintsts().read();
            if ints.usbrst() {
                trace!("reset");

                // Deconfigure everything.
                for i in 0..T::ENDPOINT_COUNT {
                    r.diepctl(i).write(|w| w.set_epdis(true));
                }

                r.gintsts().write(|w| w.set_usbrst(true)); // clear
                Self::restore_irqs();
            }
            if ints.enumdne() {
                trace!("enumdne");

                // Configure FIFOs
                // TODO unhardcode
                r.grxfsiz().write(|w| w.set_rxfd(64));
                r.dieptxf0().write(|w| {
                    w.set_sa(64);
                    w.set_fd(64);
                });

                // Flush RX/TX FIFO's
                r.grstctl().modify(|w| {
                    w.set_rxfflsh(true);
                    w.set_txfflsh(true);
                    w.set_txfnum(0b10000); // flush all
                });
                while {
                    let reg = r.grstctl().read();
                    reg.rxfflsh() || reg.txfflsh()
                } {}

                // Enable EP IRQs
                r.daintmsk().write_value(regs::Daintmsk(0xFFFF_FFFF));

                r.gintsts().write(|w| w.set_enumdne(true)); // clear
                Self::restore_irqs();

                return Poll::Ready(Event::Reset);
            }
            if ints.usbsusp() {
                trace!("suspend");
                r.gintsts().write(|w| w.set_usbsusp(true)); // clear
                Self::restore_irqs();
                return Poll::Ready(Event::Suspend);
            }
            if ints.wkupint() {
                trace!("resume");
                r.gintsts().write(|w| w.set_wkupint(true)); // clear
                Self::restore_irqs();
                return Poll::Ready(Event::Resume);
            }
            Poll::Pending
        })
        .await
    }

    #[inline]
    fn set_address(&mut self, addr: u8) {
        trace!("setting addr: {}", addr);
        unsafe {
            T::regs().dcfg().modify(|w| {
                w.set_dad(addr);
            });
        }
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        trace!("endpoint_set_stalled: {:x} {}", ep_addr, stalled);

        if ep_addr.index() >= T::ENDPOINT_COUNT {
            warn!("endpoint_set_stalled index {} out of range", ep_addr.index());
            return;
        }

        let regs = T::regs();
        match ep_addr.direction() {
            Direction::Out => unsafe {
                regs.doepctl(ep_addr.index()).modify(|w| {
                    w.set_stall(stalled);
                });
                T::state().ep_out_wakers[ep_addr.index()].wake();
            },
            Direction::In => unsafe {
                regs.diepctl(ep_addr.index()).modify(|w| {
                    w.set_stall(stalled);
                });
                T::state().ep_in_wakers[ep_addr.index()].wake();
            },
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        if ep_addr.index() >= T::ENDPOINT_COUNT {
            warn!("endpoint_is_stalled index {} out of range", ep_addr.index());
            return true;
        }

        let regs = T::regs();
        match ep_addr.direction() {
            Direction::Out => unsafe { regs.doepctl(ep_addr.index()).read().stall() },
            Direction::In => unsafe { regs.diepctl(ep_addr.index()).read().stall() },
        }
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        trace!("set_enabled {:x} {}", ep_addr, enabled);

        if ep_addr.index() >= T::ENDPOINT_COUNT {
            warn!("endpoint_set_enabled index {} out of range", ep_addr.index());
            return;
        }

        let regs = T::regs();
        match ep_addr.direction() {
            Direction::Out => unsafe { regs.doepctl(ep_addr.index()).modify(|w| w.set_epena(enabled)) },
            Direction::In => unsafe { regs.diepctl(ep_addr.index()).modify(|w| w.set_epena(enabled)) },
        }
    }

    async fn enable(&mut self) {
        trace!("enable");
    }

    async fn disable(&mut self) {
        trace!("disable");
    }

    fn force_reset(&mut self) -> Result<(), Unsupported> {
        trace!("force_reset");
        Err(Unsupported)
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

trait Dir {
    fn dir() -> Direction;
    // fn waker(i: usize) -> &'static AtomicWaker;
}

pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }

    // #[inline]
    // fn waker(i: usize) -> &'static AtomicWaker {
    //     &EP_IN_WAKERS[i]
    // }
}

pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }

    // #[inline]
    // fn waker(i: usize) -> &'static AtomicWaker {
    //     &EP_OUT_WAKERS[i]
    // }
}

pub struct Endpoint<'d, T: Instance, D> {
    _phantom: PhantomData<(&'d mut T, D)>,
    info: EndpointInfo,
    //buf: EndpointBuffer<T>,
}

impl<'d, T: Instance> embassy_usb_driver::Endpoint for Endpoint<'d, T, In> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        trace!("wait_enabled OUT WAITING");
        // todo
        trace!("wait_enabled OUT OK");
    }
}

impl<'d, T: Instance> embassy_usb_driver::Endpoint for Endpoint<'d, T, Out> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        trace!("wait_enabled OUT WAITING");
        // todo
        poll_fn(|_| Poll::<()>::Pending).await;
        trace!("wait_enabled OUT OK");
    }
}

impl<'d, T: Instance> embassy_usb_driver::EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        trace!("READ WAITING, buf.len() = {}", buf.len());
        // todo
        poll_fn(|_| Poll::<()>::Pending).await;
        let rx_len = 0;
        trace!("READ OK, rx_len = {}", rx_len);
        Ok(rx_len)
    }
}

impl<'d, T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'d, T, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        let index = self.info.addr.index();

        trace!("WRITE WAITING");

        // todo
        poll_fn(|_| Poll::<()>::Pending).await;

        trace!("WRITE OK");

        Ok(())
    }
}

pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    max_packet_size: u16,
    ep_in: Endpoint<'d, T, In>,
    ep_out: Endpoint<'d, T, Out>,
}

impl<'d, T: Instance> embassy_usb_driver::ControlPipe for ControlPipe<'d, T> {
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        loop {
            trace!("SETUP read waiting");
            // TODO
            poll_fn(|_| Poll::<()>::Pending).await;
            trace!("SETUP read ok");
            return [0; 8];
        }
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, last: bool) -> Result<usize, EndpointError> {
        let regs = T::regs();
        // TODO
        let rx_len = 0;
        Ok(rx_len)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        trace!("control: data_in");

        if data.len() > self.ep_in.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        let regs = T::regs();
        // TODO
        poll_fn(|_| Poll::<()>::Pending).await;

        trace!("WRITE OK");

        Ok(())
    }

    async fn accept(&mut self) {
        let regs = T::regs();
        trace!("control: accept");

        // TODO
        poll_fn(|_| Poll::<()>::Pending).await;

        trace!("control: accept OK");
    }

    async fn reject(&mut self) {
        let regs = T::regs();
        trace!("control: reject");

        // TODO
        poll_fn(|_| Poll::<()>::Pending).await;
    }
}

fn to_eptyp(ep_type: EndpointType) -> vals::Eptyp {
    match ep_type {
        EndpointType::Control => vals::Eptyp::CONTROL,
        EndpointType::Isochronous => vals::Eptyp::ISOCHRONOUS,
        EndpointType::Bulk => vals::Eptyp::BULK,
        EndpointType::Interrupt => vals::Eptyp::INTERRUPT,
    }
}
