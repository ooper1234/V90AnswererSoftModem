//! V.92 PCM upstream data mode (§6.4): the analogue modem's transmitter, for
//! our test modem, and the digital modem's receiver.

pub mod params;
pub mod modulus;
pub mod precoder;
pub mod trellis;
pub mod transmitter;
pub mod receiver;
pub mod design;
pub mod training;

#[cfg(test)]
pub(crate) mod fixture {
    use super::params::{Constellation, TxFilters, UpParams};
    use crate::v34::trellis::Code;
    use crate::v90::ucode::{linear, Law};

    /// drn 7 (K 48), every modulus 16, one set of 32 mu-law levels (Ucodes
    /// 40, 42, ..., 102), identity filters, G 1, 16 states.
    pub(crate) fn params() -> UpParams {
        let points: Vec<u16> = (40..=102).step_by(2).map(|u| linear(Law::Mu, u) as u16).collect();
        UpParams {
            k: 48,
            moduli: [16; 12],
            sets: vec![Constellation::from_positive(&points)],
            intervals: [0; 6],
            code: Code::States16,
            filters: TxFilters::identity(),
            gain: 1.0,
        }
    }

    /// Frames of pseudo-random bits.
    pub(crate) fn frames(n: usize, k: u32, seed: u64) -> Vec<Vec<bool>> {
        let mut x = seed | 1;
        (0..n).map(|_| (0..k).map(|_| { x ^= x << 13; x ^= x >> 7; x ^= x << 17; x & 1 == 1 }).collect()).collect()
    }
}
