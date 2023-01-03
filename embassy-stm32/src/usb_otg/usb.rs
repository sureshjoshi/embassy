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

const EP_COUNT: usize = 6; // TODO unhardcode

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static EP_IN_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];
static EP_OUT_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];

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

pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
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
                w.set_fdmod(true);
                w.set_physel(true); // internal FS PHY
            });

            // Soft-reset
            while !r.grstctl().read().ahbidl() {}
            r.grstctl().write(|w| w.set_csrst(true));
            while r.grstctl().read().csrst() {}

            r.gccfg().write(|w| {
                w.set_pwrdwn(true); // FS PHY enabled.
            });

            // BVALOVAL=1, BVALOEN=1
            r.gotgctl().write(|w| w.0 = 0x00c0);

            // Enable PHY clk.
            r.pcgcctl().write(|w| {
                w.set_stppclk(false);
            });

            // Soft disconnect.
            r.dctl().write(|w| w.set_sdis(true));

            // Set speed.
            r.dcfg().write(|w| {
                w.set_pfivl(0);
                w.set_dspd(0b11); // FS
            });

            // setup irqs
            r.gintsts().write_value(regs::Gintsts(0xFFFF_FFFF)); // clear all
            r.diepmsk().write(|w| {
                w.set_xfrcm(true);
            });
            Bus::<T>::restore_irqs();
            r.gahbcfg().write(|w| {
                w.set_gint(true);
            });

            // Connect
            r.dctl().write(|w| w.set_sdis(false));
        }

        Self {
            phantom: PhantomData,
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
            _phy_type: PhyType::ExternalHighSpeed,
        }
    }

    fn on_interrupt(_: *mut ()) {
        unsafe {
            let r = T::regs();
            let ints = r.gintsts().read();
            if ints.wkupint() || ints.usbsusp() || ints.usbrst() || ints.enumdne() {
                r.gintmsk().write(|_| {});
                BUS_WAKER.wake();
            }

            if ints.rxflvl() {
                let x = r.grxstsp_device().read();
                match x.pktsts() {
                    vals::Pktstsd::OUT_NAK => trace!("OUT_NAK"),
                    vals::Pktstsd::OUT_DATA_RX => trace!("OUT_DATA_RX"),
                    vals::Pktstsd::OUT_DATA_DONE => trace!("OUT_DATA_DONE"),
                    vals::Pktstsd::SETUP_DATA_DONE => trace!("SETUP_DATA_DONE"),
                    vals::Pktstsd::SETUP_DATA_RX => trace!("SETUP_DATA_RX"),
                    x => trace!("unknown PKTSTS: {}", x.0),
                }
            }
        }
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

        let index = 0;
        trace!("  index={}", index);

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
                //w.set_iepint(true);
                w.set_rxflvlm(true);
            });
        }
    }
}

impl<'d, T: Instance> embassy_usb_driver::Bus for Bus<'d, T> {
    async fn poll(&mut self) -> Event {
        poll_fn(move |cx| unsafe {
            let r = T::regs();

            BUS_WAKER.register(cx.waker());

            let ints = r.gintsts().read();
            if ints.usbrst() {
                trace!("reset");

                // Deconfigure everything.
                r.diepctl0().write(|w| w.set_epdis(true));
                for i in 1..EP_COUNT {
                    r.diepctl(i - 1).write(|w| w.set_epdis(true));
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

                // Flush fifos
                r.grstctl().write(|w| {
                    w.set_rxfflsh(true);
                    w.set_txfflsh(true);
                });
                loop {
                    let g = r.grstctl().read();
                    if !g.rxfflsh() && !g.txfflsh() {
                        break;
                    }
                }

                // Enable EP IRQs
                r.daintmsk().write_value(regs::Daintmsk(0xFFFF_FFFF));

                // Configure control pipe
                r.dieptsiz0().write(|w| {
                    w.set_pktcnt(0);
                    w.set_xfrsiz(64);
                });
                r.diepctl0().write(|w| {
                    w.set_mpsiz(0b00); // 64 byte
                    w.set_snak(true);
                });
                r.doeptsiz0().write(|w| {
                    w.set_stupcnt(1);
                    w.set_pktcnt(true);
                    w.set_xfrsiz(64);
                });
                r.doepctl0().write(|w| {
                    w.set_mpsiz(0b00); // 64 byte
                    w.set_epena(true);
                    w.set_cnak(true);
                });

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
        let regs = T::regs();
        trace!("setting addr: {}", addr);
        // TODO
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        // TODO
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        // TODO
        false
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        trace!("set_enabled {:x} {}", ep_addr, enabled);
        // TODO
    }

    async fn enable(&mut self) {}

    async fn disable(&mut self) {}

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

trait Dir {
    fn dir() -> Direction;
    fn waker(i: usize) -> &'static AtomicWaker;
}

pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }

    #[inline]
    fn waker(i: usize) -> &'static AtomicWaker {
        &EP_IN_WAKERS[i]
    }
}

pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }

    #[inline]
    fn waker(i: usize) -> &'static AtomicWaker {
        &EP_OUT_WAKERS[i]
    }
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
