//! # Embassy nRF HAL
//!
//! HALs implement safe, idiomatic Rust APIs to use the hardware capabilities, so raw register manipulation is not needed.
//!
//! The Embassy nRF HAL targets the Nordic Semiconductor nRF family of hardware. The HAL implements both blocking and async APIs
//! for many peripherals. The benefit of using the async APIs is that the HAL takes care of waiting for peripherals to
//! complete operations in low power mod and handling interrupts, so that applications can focus on more important matters.
//!
//! ## EasyDMA considerations
//!
//! On nRF chips, peripherals can use the so called EasyDMA feature to offload the task of interacting
//! with peripherals. It takes care of sending/receiving data over a variety of bus protocols (TWI/I2C, UART, SPI).
//! However, EasyDMA requires the buffers used to transmit and receive data to reside in RAM. Unfortunately, Rust
//! slices will not always do so. The following example using the SPI peripheral shows a common situation where this might happen:
//!
//! ```no_run
//! // As we pass a slice to the function whose contents will not ever change,
//! // the compiler writes it into the flash and thus the pointer to it will
//! // reference static memory. Since EasyDMA requires slices to reside in RAM,
//! // this function call will fail.
//! let result = spim.write_from_ram(&[1, 2, 3]);
//! assert_eq!(result, Err(Error::DMABufferNotInDataMemory));
//!
//! // The data is still static and located in flash. However, since we are assigning
//! // it to a variable, the compiler will load it into memory. Passing a reference to the
//! // variable will yield a pointer that references dynamic memory, thus making EasyDMA happy.
//! // This function call succeeds.
//! let data = [1, 2, 3];
//! let result = spim.write_from_ram(&data);
//! assert!(result.is_ok());
//! ```
//!
//! Each peripheral struct which uses EasyDMA ([`Spim`](spim::Spim), [`Uarte`](uarte::Uarte), [`Twim`](twim::Twim)) has two variants of their mutating functions:
//! - Functions with the suffix (e.g. [`write_from_ram`](spim::Spim::write_from_ram), [`transfer_from_ram`](spim::Spim::transfer_from_ram)) will return an error if the passed slice does not reside in RAM.
//! - Functions without the suffix (e.g. [`write`](spim::Spim::write), [`transfer`](spim::Spim::transfer)) will check whether the data is in RAM and copy it into memory prior to transmission.
//!
//! Since copying incurs a overhead, you are given the option to choose from `_from_ram` variants which will
//! fail and notify you, or the more convenient versions without the suffix which are potentially a little bit
//! more inefficient. Be aware that this overhead is not only in terms of instruction count but also in terms of memory usage
//! as the methods without the suffix will be allocating a statically sized buffer (up to 512 bytes for the nRF52840).
//!
//! Note that the methods that read data like [`read`](spim::Spim::read) and [`transfer_in_place`](spim::Spim::transfer_in_place) do not have the corresponding `_from_ram` variants as
//! mutable slices always reside in RAM.

#![no_std]
#![cfg_attr(
    feature = "nightly",
    feature(type_alias_impl_trait, async_fn_in_trait, impl_trait_projections)
)]
#![cfg_attr(feature = "nightly", allow(incomplete_features))]

#[cfg(not(any(
    feature = "nrf51",
    feature = "nrf52805",
    feature = "nrf52810",
    feature = "nrf52811",
    feature = "nrf52820",
    feature = "nrf52832",
    feature = "nrf52833",
    feature = "nrf52840",
    feature = "nrf5340-app-s",
    feature = "nrf5340-app-ns",
    feature = "nrf5340-net",
    feature = "nrf9160-s",
    feature = "nrf9160-ns",
)))]
compile_error!("No chip feature activated. You must activate exactly one of the following features: nrf52810, nrf52811, nrf52832, nrf52833, nrf52840");

#[cfg(all(feature = "reset-pin-as-gpio", not(feature = "_nrf52")))]
compile_error!("feature `reset-pin-as-gpio` is only valid for nRF52 series chips.");

#[cfg(all(feature = "nfc-pins-as-gpio", not(feature = "_nrf52")))]
compile_error!("feature `nfc-pins-as-gpio` is only valid for nRF52 series chips.");

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;
pub(crate) mod util;

#[cfg(feature = "_time-driver")]
mod time_driver;

#[cfg(feature = "nightly")]
pub mod buffered_uarte;
pub mod gpio;
#[cfg(feature = "gpiote")]
pub mod gpiote;
#[cfg(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))]
pub mod i2s;
pub mod nvmc;
#[cfg(any(
    feature = "nrf52810",
    feature = "nrf52811",
    feature = "nrf52833",
    feature = "nrf52840",
    feature = "_nrf9160"
))]
pub mod pdm;
pub mod ppi;
#[cfg(not(any(feature = "nrf52805", feature = "nrf52820", feature = "_nrf5340-net")))]
pub mod pwm;
#[cfg(not(any(feature = "nrf51", feature = "_nrf9160", feature = "_nrf5340")))]
pub mod qdec;
#[cfg(feature = "nrf52840")]
pub mod qspi;
#[cfg(not(any(feature = "_nrf5340", feature = "_nrf9160")))]
pub mod rng;
#[cfg(not(any(feature = "nrf52820", feature = "_nrf5340-net")))]
pub mod saadc;
pub mod spim;
pub mod spis;
#[cfg(not(any(feature = "_nrf5340", feature = "_nrf9160")))]
pub mod temp;
pub mod timer;
pub mod twim;
pub mod twis;
pub mod uarte;
#[cfg(any(
    feature = "_nrf5340-app",
    feature = "nrf52820",
    feature = "nrf52833",
    feature = "nrf52840"
))]
#[cfg(feature = "nightly")]
pub mod usb;
#[cfg(not(feature = "_nrf5340"))]
pub mod wdt;

// This mod MUST go last, so that it sees all the `impl_foo!` macros
#[cfg_attr(feature = "nrf52805", path = "chips/nrf52805.rs")]
#[cfg_attr(feature = "nrf52810", path = "chips/nrf52810.rs")]
#[cfg_attr(feature = "nrf52811", path = "chips/nrf52811.rs")]
#[cfg_attr(feature = "nrf52820", path = "chips/nrf52820.rs")]
#[cfg_attr(feature = "nrf52832", path = "chips/nrf52832.rs")]
#[cfg_attr(feature = "nrf52833", path = "chips/nrf52833.rs")]
#[cfg_attr(feature = "nrf52840", path = "chips/nrf52840.rs")]
#[cfg_attr(feature = "_nrf5340-app", path = "chips/nrf5340_app.rs")]
#[cfg_attr(feature = "_nrf5340-net", path = "chips/nrf5340_net.rs")]
#[cfg_attr(feature = "_nrf9160", path = "chips/nrf9160.rs")]
mod chip;

pub use chip::EASY_DMA_SIZE;

pub mod interrupt {
    //! nRF interrupts for cortex-m devices.
    pub use cortex_m::interrupt::{CriticalSection, Mutex};
    pub use embassy_cortex_m::interrupt::*;

    pub use crate::chip::irqs::*;
}

// Reexports

#[cfg(feature = "unstable-pac")]
pub use chip::pac;
#[cfg(not(feature = "unstable-pac"))]
pub(crate) use chip::pac;
pub use chip::{peripherals, Peripherals};
pub use embassy_cortex_m::executor;
pub use embassy_cortex_m::interrupt::_export::interrupt;
pub use embassy_hal_common::{into_ref, Peripheral, PeripheralRef};

pub mod config {
    //! Configuration options used when initializing the HAL.

    /// High frequency clock source.
    pub enum HfclkSource {
        /// Internal source
        Internal,
        /// External source from xtal.
        ExternalXtal,
    }

    /// Low frequency clock source
    pub enum LfclkSource {
        /// Internal RC oscillator
        InternalRC,
        /// Synthesized from the high frequency clock source.
        #[cfg(not(any(feature = "_nrf5340", feature = "_nrf9160")))]
        Synthesized,
        /// External source from xtal.
        ExternalXtal,
        /// External source from xtal with low swing applied.
        #[cfg(not(any(feature = "_nrf5340", feature = "_nrf9160")))]
        ExternalLowSwing,
        /// External source from xtal with full swing applied.
        #[cfg(not(any(feature = "_nrf5340", feature = "_nrf9160")))]
        ExternalFullSwing,
    }

    /// SWD access port protection setting.
    #[non_exhaustive]
    pub enum Debug {
        /// Debugging is allowed (APPROTECT is disabled). Default.
        Allowed,
        /// Debugging is not allowed (APPROTECT is enabled).
        Disallowed,
        /// APPROTECT is not configured (neither to enable it or disable it).
        /// This can be useful if you're already doing it by other means and
        /// you don't want embassy-nrf to touch UICR.
        NotConfigured,
    }

    /// Configuration for peripherals. Default configuration should work on any nRF chip.
    #[non_exhaustive]
    pub struct Config {
        /// High frequency clock source.
        pub hfclk_source: HfclkSource,
        /// Low frequency clock source.
        pub lfclk_source: LfclkSource,
        /// GPIOTE interrupt priority. Should be lower priority than softdevice if used.
        #[cfg(feature = "gpiote")]
        pub gpiote_interrupt_priority: crate::interrupt::Priority,
        /// Time driver interrupt priority. Should be lower priority than softdevice if used.
        #[cfg(feature = "_time-driver")]
        pub time_interrupt_priority: crate::interrupt::Priority,
        pub debug: Debug,
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                // There are hobby nrf52 boards out there without external XTALs...
                // Default everything to internal so it Just Works. User can enable external
                // xtals if they know they have them.
                hfclk_source: HfclkSource::Internal,
                lfclk_source: LfclkSource::InternalRC,
                #[cfg(feature = "gpiote")]
                gpiote_interrupt_priority: crate::interrupt::Priority::P0,
                #[cfg(feature = "_time-driver")]
                time_interrupt_priority: crate::interrupt::Priority::P0,

                // In NS mode, default to NotConfigured, assuming the S firmware will do it.
                #[cfg(feature = "_ns")]
                debug: Debug::NotConfigured,
                #[cfg(not(feature = "_ns"))]
                debug: Debug::Allowed,
            }
        }
    }
}

#[cfg(feature = "_nrf9160")]
mod consts {
    pub const UICR_APPROTECT: *mut u32 = 0x00FF8000 as *mut u32;
    pub const UICR_SECUREAPPROTECT: *mut u32 = 0x00FF802C as *mut u32;
    pub const APPROTECT_ENABLED: u32 = 0x0000_0000;
}

#[cfg(feature = "_nrf5340-app")]
mod consts {
    pub const UICR_APPROTECT: *mut u32 = 0x00FF8000 as *mut u32;
    pub const UICR_SECUREAPPROTECT: *mut u32 = 0x00FF801C as *mut u32;
    pub const APPROTECT_ENABLED: u32 = 0x0000_0000;
    pub const APPROTECT_DISABLED: u32 = 0x50FA50FA;
}

#[cfg(feature = "_nrf5340-net")]
mod consts {
    pub const UICR_APPROTECT: *mut u32 = 0x01FF8000 as *mut u32;
    pub const APPROTECT_ENABLED: u32 = 0x0000_0000;
    pub const APPROTECT_DISABLED: u32 = 0x50FA50FA;
}

#[cfg(feature = "_nrf52")]
#[allow(unused)]
mod consts {
    pub const UICR_PSELRESET1: *mut u32 = 0x10001200 as *mut u32;
    pub const UICR_PSELRESET2: *mut u32 = 0x10001204 as *mut u32;
    pub const UICR_NFCPINS: *mut u32 = 0x1000120C as *mut u32;
    pub const UICR_APPROTECT: *mut u32 = 0x10001208 as *mut u32;
    pub const APPROTECT_ENABLED: u32 = 0x0000_0000;
    pub const APPROTECT_DISABLED: u32 = 0x0000_005a;
}

unsafe fn uicr_write(address: *mut u32, value: u32) -> bool {
    let curr_val = address.read_volatile();
    if curr_val == value {
        return false;
    }

    // Writing to UICR can only change `1` bits to `0` bits.
    // If this write would change `0` bits to `1` bits, we can't do it.
    // It is only possible to do when erasing UICR, which is forbidden if
    // APPROTECT is enabled.
    if (!curr_val) & value != 0 {
        panic!("Cannot write UICR address={:08x} value={:08x}", address as u32, value)
    }

    let nvmc = &*pac::NVMC::ptr();
    nvmc.config.write(|w| w.wen().wen());
    while nvmc.ready.read().ready().is_busy() {}
    address.write_volatile(value);
    while nvmc.ready.read().ready().is_busy() {}
    nvmc.config.reset();
    while nvmc.ready.read().ready().is_busy() {}

    true
}

/// Initialize peripherals with the provided configuration. This should only be called once at startup.
pub fn init(config: config::Config) -> Peripherals {
    // Do this first, so that it panics if user is calling `init` a second time
    // before doing anything important.
    let peripherals = Peripherals::take();

    let mut needs_reset = false;

    // Setup debug protection.
    match config.debug {
        config::Debug::Allowed => {
            #[cfg(feature = "_nrf52")]
            unsafe {
                let variant = (0x1000_0104 as *mut u32).read_volatile();
                // Get the letter for the build code (b'A' .. b'F')
                let build_code = (variant >> 8) as u8;

                if build_code >= b'F' {
                    // Chips with build code F and higher (revision 3 and higher) have an
                    // improved APPROTECT ("hardware and software controlled access port protection")
                    // which needs explicit action by the firmware to keep it unlocked

                    // UICR.APPROTECT = SwDisabled
                    needs_reset |= uicr_write(consts::UICR_APPROTECT, consts::APPROTECT_DISABLED);
                    // APPROTECT.DISABLE = SwDisabled
                    (0x4000_0558 as *mut u32).write_volatile(consts::APPROTECT_DISABLED);
                } else {
                    // nothing to do on older chips, debug is allowed by default.
                }
            }

            #[cfg(feature = "_nrf5340")]
            unsafe {
                let p = &*pac::CTRLAP::ptr();

                needs_reset |= uicr_write(consts::UICR_APPROTECT, consts::APPROTECT_DISABLED);
                p.approtect.disable.write(|w| w.bits(consts::APPROTECT_DISABLED));

                #[cfg(feature = "_nrf5340-app")]
                {
                    needs_reset |= uicr_write(consts::UICR_SECUREAPPROTECT, consts::APPROTECT_DISABLED);
                    p.secureapprotect.disable.write(|w| w.bits(consts::APPROTECT_DISABLED));
                }
            }

            // nothing to do on the nrf9160, debug is allowed by default.
        }
        config::Debug::Disallowed => unsafe {
            // UICR.APPROTECT = Enabled
            needs_reset |= uicr_write(consts::UICR_APPROTECT, consts::APPROTECT_ENABLED);
            #[cfg(any(feature = "_nrf5340-app", feature = "_nrf9160"))]
            {
                needs_reset |= uicr_write(consts::UICR_SECUREAPPROTECT, consts::APPROTECT_ENABLED);
            }
        },
        config::Debug::NotConfigured => {}
    }

    #[cfg(all(feature = "_nrf52", not(feature = "reset-pin-as-gpio")))]
    unsafe {
        needs_reset |= uicr_write(consts::UICR_PSELRESET1, chip::RESET_PIN);
        needs_reset |= uicr_write(consts::UICR_PSELRESET2, chip::RESET_PIN);
    }

    #[cfg(all(feature = "_nrf52", feature = "nfc-pins-as-gpio"))]
    unsafe {
        needs_reset |= uicr_write(consts::UICR_NFCPINS, 0);
    }

    if needs_reset {
        cortex_m::peripheral::SCB::sys_reset();
    }

    let r = unsafe { &*pac::CLOCK::ptr() };

    // Start HFCLK.
    match config.hfclk_source {
        config::HfclkSource::Internal => {}
        config::HfclkSource::ExternalXtal => {
            // Datasheet says this is likely to take 0.36ms
            r.events_hfclkstarted.write(|w| unsafe { w.bits(0) });
            r.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
            while r.events_hfclkstarted.read().bits() == 0 {}
        }
    }

    // Configure LFCLK.
    #[cfg(not(any(feature = "_nrf5340", feature = "_nrf9160")))]
    match config.lfclk_source {
        config::LfclkSource::InternalRC => r.lfclksrc.write(|w| w.src().rc()),
        config::LfclkSource::Synthesized => r.lfclksrc.write(|w| w.src().synth()),

        config::LfclkSource::ExternalXtal => r.lfclksrc.write(|w| w.src().xtal()),

        config::LfclkSource::ExternalLowSwing => r.lfclksrc.write(|w| {
            w.src().xtal();
            w.external().enabled();
            w.bypass().disabled();
            w
        }),
        config::LfclkSource::ExternalFullSwing => r.lfclksrc.write(|w| {
            w.src().xtal();
            w.external().enabled();
            w.bypass().enabled();
            w
        }),
    }
    #[cfg(feature = "_nrf9160")]
    match config.lfclk_source {
        config::LfclkSource::InternalRC => r.lfclksrc.write(|w| w.src().lfrc()),
        config::LfclkSource::ExternalXtal => r.lfclksrc.write(|w| w.src().lfxo()),
    }

    // Start LFCLK.
    // Datasheet says this could take 100us from synth source
    // 600us from rc source, 0.25s from an external source.
    r.events_lfclkstarted.write(|w| unsafe { w.bits(0) });
    r.tasks_lfclkstart.write(|w| unsafe { w.bits(1) });
    while r.events_lfclkstarted.read().bits() == 0 {}

    // Init GPIOTE
    #[cfg(feature = "gpiote")]
    gpiote::init(config.gpiote_interrupt_priority);

    // init RTC time driver
    #[cfg(feature = "_time-driver")]
    time_driver::init(config.time_interrupt_priority);

    // Disable UARTE (enabled by default for some reason)
    #[cfg(feature = "_nrf9160")]
    unsafe {
        (*pac::UARTE0::ptr()).enable.write(|w| w.enable().disabled());
        (*pac::UARTE1::ptr()).enable.write(|w| w.enable().disabled());
    }

    peripherals
}
