//! V.92 Phase 3 signals when PCM upstream is selected (8.5, 8.6): Ja's DIL
//! descriptor with its upstream rate mask, the digital modem's Jd, Jp and
//! Jp', and the fixed Ru and Su patterns. Nothing here is offered on a live
//! call; see `V92_PCM_UPSTREAM`.

/// CPt's rate (Table 23): "(drn + 8) x 8000/6", drn 1 to 22; 0 is cleardown.
pub fn training_rate(drn: u8) -> Option<u32> {
    (1..=22).contains(&drn).then(|| (u32::from(drn) + 8) * 8000 / 6)
}

use crate::v90::sequences::{frame, get, put, unframe, Descriptor, BLOCK};

/// Jd and Jp are both sync, two blocks, the CRC and "Fill bits: 0000".
pub const J_BITS: usize = 72;

/// Jp' (8.6.4): "12 binary zeroes".
pub const JP_PRIME_BITS: usize = 12;

/// Jd when PCM upstream is selected (Table 21). Unlike V.90's Jd, bit 47
/// tells Jd from Jp and bit 48 is reserved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpJd {
    /// Bit n: downstream drn n + 1 enabled, 28 000 in bit 0 to 56 000 in
    /// bit 21, spread over bits 18:33 and 35:40.
    pub rates: u32,
    /// Bits 49:50: "the digital modem's maximum look-ahead for spectral
    /// shaping", 1 to 3.
    pub lookahead: u8,
}

/// Jp (Table 22): the digital modem asks the analogue modem to move its
/// transmitter's phase, and names the TRN2u constellations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Jp {
    /// Bits 18:33: how far Su is to be extended at the Jp to Jp' transition,
    /// a 16-bit fraction of a symbol.
    pub su_extension: u16,
    /// Bit 48: 8 points rather than 4 for CPu, E2u, SUVu and TRN2u in training.
    pub eight_point_training: bool,
    /// Bit 49: the same in rate renegotiation.
    pub eight_point_renegotiation: bool,
}

/// What a 72-bit J sequence turns out to be, by bit 47.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhaseThreeJ {
    Jd(UpJd),
    Jp(Jp),
}

fn j_bits(information: &[bool]) -> Vec<bool> {
    let mut bits = frame(information);
    bits.resize(J_BITS, false);
    bits
}

impl UpJd {
    pub fn to_bits(&self) -> Vec<bool> {
        let mut information = Vec::with_capacity(2 * BLOCK);
        put(&mut information, self.rates & 0xffff, 16);
        put(&mut information, self.rates >> 16 & 0x3f, 6);
        put(&mut information, 0, 6); // 41:46 reserved
        information.push(false); // 47: Jd
        information.push(false); // 48 reserved
        put(&mut information, u32::from(self.lookahead), 2);
        j_bits(&information)
    }
}

impl Jp {
    pub fn to_bits(&self) -> Vec<bool> {
        let mut information = Vec::with_capacity(2 * BLOCK);
        put(&mut information, u32::from(self.su_extension), 16);
        put(&mut information, 0, 12); // 35:46 reserved
        information.push(true); // 47: Jp
        information.push(self.eight_point_training);
        information.push(self.eight_point_renegotiation);
        information.push(false); // 50 reserved
        j_bits(&information)
    }
}

impl PhaseThreeJ {
    /// Jd or Jp from `bits`, which start at the frame sync.
    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let information = unframe(bits, 2)?;
        Some(if information[28] {
            Self::Jp(Jp {
                su_extension: get(&information, 0, 16) as u16,
                eight_point_training: information[29],
                eight_point_renegotiation: information[30],
            })
        } else {
            Self::Jd(UpJd { rates: get(&information, 0, 22), lookahead: get(&information, 30, 2) as u8 })
        })
    }
}

/// The DIL descriptor Ja repeats (Table 20): V.90's descriptor, then the
/// analogue modem's upstream rate mask in two blocks, before the CRC.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JaDescriptor {
    pub dil: Descriptor,
    /// Bit n: upstream drn n + 1 "supported and enabled in the transmitter
    /// of the analogue modem", 24 000 in bit 0 to 48 000 in bit 18.
    pub upstream_rates: u32,
}

impl JaDescriptor {
    /// One descriptor from its frame sync through the fill that makes it a
    /// multiple of 12 bits. Ja's leading 24 ones are the sender's.
    pub fn to_bits(&self) -> Vec<bool> {
        let mut information = self.dil.information();
        put(&mut information, self.upstream_rates & 0xffff, 16);
        put(&mut information, self.upstream_rates >> 16 & 0x7, 3);
        put(&mut information, 0, 13); // reserved
        let mut bits = frame(&information);
        bits.push(false); // "Fill bit: 0"
        bits.resize(bits.len().div_ceil(12) * 12, false);
        bits
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let blocks = Descriptor::information_blocks(bits)? + 2;
        let information = unframe(bits, blocks)?;
        let dil = Descriptor::from_information(&information[..(blocks - 2) * BLOCK])?;
        let mask = (blocks - 2) * BLOCK;
        let upstream_rates = get(&information, mask, 16) | get(&information, mask + BLOCK, 3) << 16;
        Some(Self { dil, upstream_rates })
    }
}

/// Ru (8.5.5) as multiples of LU; Ru-bar is its negation.
pub const RU: [f64; 6] = [1., 1., 1., -1., -1., -1.];

/// Su (8.5.6) as multiples of LU; Su-bar is its negation.
pub fn su() -> [f64; 6] {
    let s = 1.5f64.sqrt();
    [s, 0., s, -s, 0., -s]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test] fn jp_puts_each_field_where_table_22_says() {
        let b = Jp { su_extension: 0x8001, eight_point_training: true, eight_point_renegotiation: false }.to_bits();
        assert_eq!(b.len(), 72);
        assert!(b[0..17].iter().all(|&x| x));
        assert!(!b[17] && !b[34] && !b[51]);
        assert!(b[18] && b[33] && !b[19]);
        assert!(b[35..47].iter().all(|&x| !x));
        assert!(b[47]);
        assert!(b[48] && !b[49] && !b[50]);
        assert!(b[68..72].iter().all(|&x| !x));
    }

    #[test] fn jd_has_identifier_zero_and_no_16_point_flags() {
        let b = UpJd { rates: 1 | 1 << 21, lookahead: 3 }.to_bits();
        assert_eq!(b.len(), 72);
        assert!(b[18] && b[40] && !b[41]);
        assert!(!b[47] && !b[48]);
        assert!(b[49] && b[50]);
    }

    #[test] fn bit_47_decides_jd_or_jp() {
        let jp = Jp { su_extension: 0x1234, eight_point_training: false, eight_point_renegotiation: true };
        assert_eq!(PhaseThreeJ::from_bits(&jp.to_bits()), Some(PhaseThreeJ::Jp(jp)));
        let jd = UpJd { rates: 0x3f_ffff, lookahead: 1 };
        assert_eq!(PhaseThreeJ::from_bits(&jd.to_bits()), Some(PhaseThreeJ::Jd(jd)));
        let mut bad = jd.to_bits(); bad[20] ^= true;
        assert_eq!(PhaseThreeJ::from_bits(&bad), None);
    }

    #[test] fn ja_with_no_dil_is_276_bits_with_the_masks_at_222_and_239() {
        let ja = JaDescriptor { dil: Descriptor::none(), upstream_rates: 1 | 1 << 15 | 1 << 18 };
        let b = ja.to_bits();
        assert_eq!(b.len(), 276);
        assert!(!b[221] && !b[238] && !b[255]);
        assert!(b[222] && b[237]);
        assert!(!b[239] && !b[240] && b[241]);
        assert!(b[242..255].iter().all(|&x| !x));
        assert!(b[272..].iter().all(|&x| !x));
        assert_eq!(JaDescriptor::from_bits(&b), Some(ja));
    }

    #[test] fn ja_keeps_a_real_dil_intact() {
        let dil = Descriptor {
            signs: (0..20).map(|i| i % 3 == 0).collect(),
            training: (0..5).map(|i| i != 2).collect(),
            h: [1, 2, 3, 4, 5, 6, 7, 8],
            refs: [10, 20, 30, 40, 50, 60, 70, 80],
            ucodes: vec![79, 90, 101, 112, 120],
        };
        let ja = JaDescriptor { dil, upstream_rates: (1 << 19) - 1 };
        let b = ja.to_bits();
        assert_eq!(b.len() % 12, 0);
        assert_eq!(JaDescriptor::from_bits(&b), Some(ja.clone()));
        let mut bad = b.clone(); bad[30] ^= true;
        assert_eq!(JaDescriptor::from_bits(&bad), None);
    }

    #[test] fn su_has_the_power_of_ru() {
        let p = |x: &[f64]| x.iter().map(|v| v * v).sum::<f64>();
        assert!((p(&su()) - p(&RU)).abs() < 1e-12);
        assert_eq!(RU, [1., 1., 1., -1., -1., -1.]);
        let s = 1.5f64.sqrt();
        assert_eq!(su(), [s, 0., s, -s, 0., -s]);
    }
}
