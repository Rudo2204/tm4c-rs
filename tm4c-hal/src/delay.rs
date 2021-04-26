//! Code for busy-waiting

use crate::{
    sysctl::Clocks,
    time::{Hertz, U32Ext},
};
use cortex_m::peripheral::{syst::SystClkSource, SYST};
use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    timer::CountDown,
};
use nb::block;

/// System timer (SysTick) as a delay provider
pub struct Delay {
    sysclk: Hertz,
    syst: SYST,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);

        Delay {
            syst,
            sysclk: clocks.sysclk,
        }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(cast::u32(ms));
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(cast::u32(ms));
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // Tricky to get this to not overflow
        let mut rvr = us * (self.sysclk.0 / 1_000_000);
        rvr += (us * ((self.sysclk.0 % 1_000_000) / 1_000)) / 1_000;
        rvr += (us * (self.sysclk.0 % 1_000)) / 1_000_000;

        while rvr >= 1 << 24 {
            self.syst.set_reload((1 << 24) - 1);
            self.syst.clear_current();
            self.syst.enable_counter();
            while !self.syst.has_wrapped() {}
            self.syst.disable_counter();
            rvr -= 1 << 24;
        }

        assert!(rvr < (1 << 24));
        self.syst.set_reload(rvr);
        self.syst.clear_current();
        self.syst.enable_counter();
        while !self.syst.has_wrapped() {}
        self.syst.disable_counter();
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(cast::u32(us))
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(cast::u32(us))
    }
}

/// CountDown Timer as a delay provider
pub struct DelayFromCountDownTimer<T>(T);

impl<T> DelayFromCountDownTimer<T> {
    /// Creates delay provider from a CountDown timer
    pub fn new(timer: T) -> Self {
        Self(timer)
    }

    /// Releases the Timer
    pub fn free(self) -> T {
        self.0
    }
}

macro_rules! impl_delay_from_count_down_timer  {
    ($(($Delay:ident, $delay:ident, $num:expr)),+) => {
        $(

            impl<T> $Delay<u32> for DelayFromCountDownTimer<T>
            where
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u32) {
                    let mut time_left = t;

                    // Due to the LpTimer having only a 3 bit scaler, it is
                    // possible that the max timeout we can set is
                    // (128 * 65536) / clk_hz milliseconds.
                    // Assuming the fastest clk_hz = 80Mhz this is roughly ~105ms,
                    // or a frequency of ~9.5Hz. We use a 10Hz frequency for each
                    // loop step here to ensure that we stay within these bounds.
                    let looping_delay = $num / 10;
                    let looping_delay_hz = Hertz($num / looping_delay);

                    self.0.start(looping_delay_hz);
                    while time_left > looping_delay {
                        block!(self.0.wait()).ok();
                        time_left = time_left - looping_delay;
                    }

                    if time_left > 0 {
                        self.0.start(($num / time_left).hz());
                        block!(self.0.wait()).ok();
                    }
                }
            }

            impl<T> $Delay<u16> for DelayFromCountDownTimer<T>
            where
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u16) {
                    self.$delay(t as u32);
                }
            }

            impl<T> $Delay<u8> for DelayFromCountDownTimer<T>
            where
                T: CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u8) {
                    self.$delay(t as u32);
                }
            }
        )+
    }
}

impl_delay_from_count_down_timer! {
    (DelayMs, delay_ms, 1_000),
    (DelayUs, delay_us, 1_000_000)
}
