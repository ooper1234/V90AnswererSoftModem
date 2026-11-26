//! The data-mode parameters CPd gives the analogue modem (Table 30), checked
//! once so the transmitter and receiver never meet a frame they cannot carry.

use crate::v34::trellis::Code;
use crate::v92::phase4::{Cpd, Filters, UpTrellis};

/// One constellation set: the N = 2 x LC points a(eta), -N/2 <= eta < N/2,
/// negative indices for negative points (6.4.2).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Constellation {
    positive: Vec<i32>,
}

impl Constellation {
    /// From CPd's positive linear values, smallest first.
    pub fn from_positive(points: &[u16]) -> Self {
        Self { positive: points.iter().map(|&p| i32::from(p)).collect() }
    }

    pub fn n(&self) -> i32 {
        2 * self.positive.len() as i32
    }

    /// a(eta); eta outside -N/2..N/2 is a caller's error and gives 0.
    pub fn a(&self, eta: i32) -> i32 {
        let lc = self.positive.len() as i32;
        match eta {
            0.. if eta < lc => self.positive[eta as usize],
            ..0 if eta >= -lc => -self.positive[(-eta - 1) as usize],
            _ => 0,
        }
    }

    /// The largest magnitude.
    pub fn max(&self) -> i32 {
        self.positive.iter().copied().max().unwrap_or(0)
    }
}

/// The precoder and prefilter coefficients as numbers: z1(1..), p1(1..),
/// z2(0..) and p2(1..).
#[derive(Debug, Clone, PartialEq)]
pub struct TxFilters {
    pub z1: Vec<f64>,
    pub p1: Vec<f64>,
    pub z2: Vec<f64>,
    pub p2: Vec<f64>,
}

impl TxFilters {
    /// z in Q0.15, p in Q1.14 (Table 30).
    pub fn from_cpd(f: &Filters) -> Self {
        let z = |v: &[i16]| v.iter().map(|&c| f64::from(c) / 32768.0).collect();
        let p = |v: &[i16]| v.iter().map(|&c| f64::from(c) / 16384.0).collect();
        Self { z1: z(&f.z1), p1: p(&f.p1), z2: z(&f.z2), p2: p(&f.p2) }
    }

    /// No precoding and a prefilter that passes x(n) through.
    pub fn identity() -> Self {
        Self { z1: vec![], p1: vec![], z2: vec![1.0], p2: vec![] }
    }
}

/// Why a CPd cannot drive data mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamError {
    /// drn 0.
    Cleardown,
    MissingModuli,
    MissingConstellations,
    ZeroModulus { interval: usize },
    /// The product of the moduli is below 2^K.
    TooFewMessages { k: u32 },
    /// Some K_i of this interval would have no point to send.
    EmptyClass { interval: usize },
    /// A point louder than the codec's loudest level (receiver only).
    BeyondCodec,
}

/// Everything the transmitter and receiver need for data mode.
#[derive(Debug, Clone, PartialEq)]
pub struct UpParams {
    /// Bits per data frame, 2 x (drn + 17).
    pub k: u32,
    pub moduli: [u16; 12],
    pub sets: Vec<Constellation>,
    /// The set of intervals j and j + 6.
    pub intervals: [u8; 6],
    pub code: Code,
    pub filters: TxFilters,
    /// G, from 4 x G in Q0.16.
    pub gain: f64,
}

impl UpParams {
    pub fn from_cpd(cpd: &Cpd) -> Result<Self, ParamError> {
        if cpd.drn == 0 {
            return Err(ParamError::Cleardown);
        }
        let moduli = cpd.moduli.ok_or(ParamError::MissingModuli)?.map(u16::from);
        let constellations = cpd.constellations.as_ref().ok_or(ParamError::MissingConstellations)?;
        if let Some(interval) = moduli.iter().position(|&m| m == 0) {
            return Err(ParamError::ZeroModulus { interval });
        }
        let k = 2 * (u32::from(cpd.drn) + 17);
        let params = Self {
            k,
            moduli,
            sets: constellations.sets.iter().map(|s| Constellation::from_positive(s)).collect(),
            intervals: constellations.intervals,
            code: match cpd.trellis {
                UpTrellis::States16 => Code::States16,
                UpTrellis::States32 => Code::States32,
                UpTrellis::States64 => Code::States64,
            },
            filters: cpd.filters.as_ref().map_or_else(TxFilters::identity, TxFilters::from_cpd),
            gain: f64::from(cpd.gain4) / 262_144.0,
        };
        if params.modulus_product() < 1u128 << k {
            return Err(ParamError::TooFewMessages { k });
        }
        for interval in 0..12 {
            let set = usize::from(params.intervals[interval % 6]);
            let n = params.sets.get(set).map_or(0, Constellation::n);
            let need = i32::from(moduli[interval]) * if interval % 4 == 3 { 2 } else { 1 };
            if n < need {
                return Err(ParamError::EmptyClass { interval });
            }
        }
        Ok(params)
    }

    /// The constellation of data frame interval 0 to 11.
    pub fn constellation(&self, interval: usize) -> &Constellation {
        &self.sets[usize::from(self.intervals[interval % 6])]
    }

    /// M, the product of M0 to M11 (Mi <= 255, so M < 2^96).
    pub fn modulus_product(&self) -> u128 {
        self.moduli.iter().map(|&m| u128::from(m)).product()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v34::trellis::Code;
    use crate::v92::phase4::{Cpd, Filters, UpConstellations, UpTrellis};

    fn cpd(drn: u8, moduli: Option<[u8; 12]>, sets: Option<Vec<Vec<u16>>>, intervals: [u8; 6]) -> Cpd {
        Cpd {
            drn,
            trellis: UpTrellis::States16,
            extend_e2u: false,
            acknowledge: false,
            gain4: 32768,
            moduli,
            filters: None,
            constellations: sets.map(|sets| UpConstellations { intervals, sets }),
        }
    }

    fn set(lc: u16) -> Vec<u16> { (1..=lc).map(|i| i * 10).collect() }

    #[test]
    fn k_is_twice_drn_plus_17() {
        for (drn, k) in [(1, 36), (7, 48), (19, 72)] {
            let p = UpParams::from_cpd(&cpd(drn, Some([128; 12]), Some(vec![set(128)]), [0; 6])).unwrap();
            assert_eq!(p.k, k);
        }
    }

    #[test]
    fn a_runs_negative_to_positive() {
        let c = Constellation::from_positive(&[10, 20, 30]);
        assert_eq!(c.n(), 6);
        assert_eq!((c.a(0), c.a(2), c.a(-1), c.a(-3)), (10, 30, -10, -30));
        assert_eq!(c.max(), 30);
    }

    #[test]
    fn coefficients_scale_by_their_q_formats() {
        let f = TxFilters::from_cpd(&Filters { z1: vec![16384], p1: vec![8192], z2: vec![], p2: vec![-16384] });
        assert_eq!((f.z1[0], f.p1[0], f.p2[0]), (0.5, 0.5, -1.0));
        let mut c = cpd(7, Some([16; 12]), Some(vec![set(32)]), [0; 6]);
        c.gain4 = 32768;
        assert_eq!(UpParams::from_cpd(&c).unwrap().gain, 0.125);
        // No filters in CPd: z2 = [1], the identity.
        assert_eq!(UpParams::from_cpd(&c).unwrap().filters.z2, vec![1.0]);
        assert_eq!(UpParams::from_cpd(&c).unwrap().code, Code::States16);
    }

    #[test]
    fn interval_names_its_set_for_i_and_i_plus_6() {
        let p = UpParams::from_cpd(&cpd(7, Some([16; 12]), Some(vec![set(16), set(32)]), [0, 1, 0, 1, 0, 1])).unwrap();
        assert_eq!(p.constellation(7).n(), 64);
        assert_eq!(p.constellation(6).n(), 32);
    }

    #[test]
    fn refusals() {
        let ok = |m, s| cpd(7, m, s, [0; 6]);
        assert_eq!(UpParams::from_cpd(&cpd(0, Some([16; 12]), Some(vec![set(32)]), [0; 6])).unwrap_err(), ParamError::Cleardown);
        assert_eq!(UpParams::from_cpd(&ok(None, Some(vec![set(32)]))).unwrap_err(), ParamError::MissingModuli);
        assert_eq!(UpParams::from_cpd(&ok(Some([16; 12]), None)).unwrap_err(), ParamError::MissingConstellations);
        let mut zero = [16; 12];
        zero[5] = 0;
        assert_eq!(UpParams::from_cpd(&ok(Some(zero), Some(vec![set(32)]))).unwrap_err(), ParamError::ZeroModulus { interval: 5 });
        assert_eq!(UpParams::from_cpd(&ok(Some([15; 12]), Some(vec![set(32)]))).unwrap_err(), ParamError::TooFewMessages { k: 48 });
        assert_eq!(UpParams::from_cpd(&ok(Some([16; 12]), Some(vec![set(8)]))).unwrap_err(), ParamError::EmptyClass { interval: 3 });
        assert!(UpParams::from_cpd(&ok(Some([16; 12]), Some(vec![set(16)]))).is_ok());
    }
}
