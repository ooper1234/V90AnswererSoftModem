//! V.92 Phase 4, rate renegotiation and fast parameter exchange signals
//! for PCM upstream (8.7, 8.8): CPt/CPu, CPus, SUVu, SUVd, CPd, RM and TRN2u.

/// How many analogue-to-digital rates CPd's drn names (Table 30).
pub const UPSTREAM_RATES: u8 = 19;

/// CPd's rate: "(drn + 17) x 8000/6", 24 000 to 48 000; 0 is cleardown.
pub fn upstream_rate(drn: u8) -> Option<u32> {
    (1..=UPSTREAM_RATES).contains(&drn).then(|| (u32::from(drn) + 17) * 8000 / 6)
}

/// CPu's rate: "(drn + 20) x 8000/6", 28 000 to 56 000 as in V.90.
pub fn downstream_rate(drn: u8) -> Option<u32> {
    (1..=22).contains(&drn).then(|| (u32::from(drn) + 20) * 8000 / 6)
}

use crate::v90::sequences::{frame, get, put, unframe, CpBody, Mask, BLOCK};
use crate::v90::sign::Redundancy;

/// Bits 19:20 of Table 23 and 24: which CP this is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CpKind {
    /// CPt: the training constellations, rate (drn + 8) x 8000/6.
    Training = 0,
    /// CPu: data mode's, rate (drn + 20) x 8000/6.
    Data = 1,
}

/// CPt and CPu (Table 23): what the analogue modem asks the digital modem
/// to send downstream with. The layout is V.90's CP but for the first two
/// blocks.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UpCp {
    pub kind: CpKind,
    /// Bits 21:25; 0 is cleardown.
    pub drn: u8,
    /// Bits 31:32: Sr.
    pub redundancy: Redundancy,
    /// Bit 33: CPd has arrived, making this CPu'.
    pub acknowledge: bool,
    /// Bit 35.
    pub a_law: bool,
    /// Bits 49:50: ld.
    pub lookahead: u8,
    /// Bits 52:67, unsigned Q3.13.
    pub trn1d_ratio: u16,
    /// a1, a2, b1, b2, signed Q1.6.
    pub shaping: [i8; 4],
    pub intervals: [u8; 6],
    pub constellations: Vec<Mask>,
    pub codec: Option<Vec<Mask>>,
}

impl Default for UpCp {
    fn default() -> Self {
        Self {
            kind: CpKind::Data,
            drn: 0,
            redundancy: Redundancy::None,
            acknowledge: false,
            a_law: false,
            lookahead: 0,
            trn1d_ratio: 1 << 13,
            shaping: [0; 4],
            intervals: [0; 6],
            constellations: vec![0],
            codec: None,
        }
    }
}

/// CPus (Table 24): a short CPu when the digital modem's parameters stay.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cpus {
    pub drn: u8,
    pub acknowledge: bool,
}

/// SUVu (Table 27).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Suvu {
    /// Bit 26: wait for CPu before sending CPd.
    pub wait_for_cpu: bool,
    /// Bits 27:31: 20 log10(L) of the prefilter output times G, signed Q2.2;
    /// -16 (-4.00) is "no measurement has been taken".
    pub level_q2_2: i8,
    /// Bit 32: a silent period is requested.
    pub silence: bool,
    /// Bit 33: CPd has arrived.
    pub acknowledge: bool,
}

/// SUVd (Table 31).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Suvd {
    pub silence: bool,
    pub acknowledge: bool,
}

/// What the digital modem can hear from the analogue modem in Phase 4,
/// told apart by bit 18 (CP 0, SUV 1) and bits 19:20.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UpInfo {
    Cp(UpCp),
    Cpus(Cpus),
    Suv(Suvu),
}

fn redundancy(spent: u32) -> Redundancy {
    match spent {
        0 => Redundancy::None,
        1 => Redundancy::One,
        2 => Redundancy::Two,
        _ => Redundancy::Three,
    }
}

/// Framed information plus the one fill bit every one of these ends with.
fn with_fill(information: &[bool]) -> Vec<bool> {
    let mut bits = frame(information);
    bits.push(false);
    bits
}

/// Extends `bits` with zeros to a multiple of `multiple` bits: 12 or 6
/// symbols' worth, depending on the sequence and its modulation.
pub fn pad(bits: &mut Vec<bool>, multiple: usize) {
    bits.resize(bits.len().div_ceil(multiple) * multiple, false);
}

/// The first block of a short sequence: bit 18 and the next 15.
fn short(information: &mut Vec<bool>, suv: bool, kind: u32) {
    information.push(suv);
    if !suv {
        put(information, kind, 2);
    }
}

impl UpCp {
    pub fn to_bits(&self) -> Vec<bool> {
        let mut information = Vec::new();
        short(&mut information, false, self.kind as u32);
        put(&mut information, u32::from(self.drn), 5);
        put(&mut information, 0, 5); // 26:30 reserved
        put(&mut information, self.redundancy.spent() as u32, 2);
        information.push(self.acknowledge);
        information.push(self.a_law);
        put(&mut information, 0, 13); // 36:48 reserved
        put(&mut information, u32::from(self.lookahead), 2);
        CpBody {
            trn1d_ratio: self.trn1d_ratio,
            shaping: self.shaping,
            intervals: self.intervals,
            constellations: self.constellations.clone(),
            codec: self.codec.clone(),
        }
        .push(&mut information);
        with_fill(&information)
    }
}

impl Cpus {
    pub fn to_bits(&self) -> Vec<bool> {
        let mut information = Vec::new();
        short(&mut information, false, 2);
        put(&mut information, u32::from(self.drn), 5);
        put(&mut information, 0, 7); // 26:32 reserved
        information.push(self.acknowledge);
        with_fill(&information)
    }
}

impl Suvu {
    pub fn to_bits(&self) -> Vec<bool> {
        let mut information = Vec::new();
        short(&mut information, true, 0);
        put(&mut information, 0, 7); // 19:25 reserved
        information.push(self.wait_for_cpu);
        put(&mut information, u32::from(self.level_q2_2 as u8) & 0x1f, 5);
        information.push(self.silence);
        information.push(self.acknowledge);
        with_fill(&information)
    }
}

impl Suvd {
    pub fn to_bits(&self) -> Vec<bool> {
        let mut information = Vec::new();
        short(&mut information, true, 0);
        put(&mut information, 0, 13); // 19:31 reserved
        information.push(self.silence);
        information.push(self.acknowledge);
        with_fill(&information)
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let information = unframe(bits, 1)?;
        information[0].then(|| Self { silence: information[14], acknowledge: information[15] })
    }
}

impl UpInfo {
    /// A CPt, CPu, CPus or SUVu out of `bits`, which start at the frame sync.
    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let first = unframe(bits, 1);
        let at = |bit: usize| bits.get(bit).copied();
        if at(18)? {
            let information = first?;
            // Sign-extend the 5-bit Q2.2 level.
            let level = ((get(&information, 9, 5) as u8) << 3) as i8 >> 3;
            return Some(Self::Suv(Suvu {
                wait_for_cpu: information[8],
                level_q2_2: level,
                silence: information[14],
                acknowledge: information[15],
            }));
        }
        let kind = u32::from(at(19)?) | u32::from(at(20)?) << 1;
        if kind == 2 {
            let information = first?;
            return Some(Self::Cpus(Cpus { drn: get(&information, 3, 5) as u8, acknowledge: information[15] }));
        }
        let kind = match kind {
            0 => CpKind::Training,
            1 => CpKind::Data,
            _ => return None,
        };
        let information = unframe(bits, CpBody::blocks(bits)?)?;
        let CpBody { trn1d_ratio, shaping, intervals, constellations, codec } = CpBody::read(&information);
        Some(Self::Cp(UpCp {
            kind,
            drn: get(&information, 3, 5) as u8,
            redundancy: redundancy(get(&information, 13, 2)),
            acknowledge: information[15],
            a_law: information[16],
            lookahead: get(&information, 30, 2) as u8,
            trn1d_ratio,
            shaping,
            intervals,
            constellations,
            codec,
        }))
    }
}

/// CPd bits 27:28: the trellis the analogue modem's transmitter is to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpTrellis {
    States16 = 0,
    States32 = 1,
    States64 = 2,
}

/// CPd's precoder and prefilter coefficients, as sent: z in signed Q0.15,
/// p in signed Q1.14. `z1` is z1(1..=LZ1), `p1` p1(1..=LP1), `z2`
/// z2(0..LZ2) and `p2` p2(1..=LP2).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Filters {
    pub z1: Vec<i16>,
    pub p1: Vec<i16>,
    pub z2: Vec<i16>,
    pub p2: Vec<i16>,
}

impl Filters {
    pub fn lengths(&self) -> [usize; 4] {
        [self.z1.len(), self.p1.len(), self.z2.len(), self.p2.len()]
    }
}

/// CPd's constellation sets. Interval i names its set for data frame
/// intervals i and i + 6. Each set lists its positive linear values,
/// smallest first; a set's length is its LC.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UpConstellations {
    pub intervals: [u8; 6],
    pub sets: Vec<Vec<u16>>,
}

/// CPd (Table 30): the digital modem's parameters for the analogue modem's
/// PCM transmitter. Each optional part is present when it is `Some`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Cpd {
    /// Bits 22:26, 0 (cleardown) to 19; see [`upstream_rate`].
    pub drn: u8,
    pub trellis: UpTrellis,
    /// Bit 29: E2u one symbol longer. Zero outside training.
    pub extend_e2u: bool,
    /// Bit 33: CPu has arrived, making this CPd'.
    pub acknowledge: bool,
    /// Bits 35:50: 4 x G, unsigned Q0.16.
    pub gain4: u16,
    /// M0 to M11.
    pub moduli: Option<[u8; 12]>,
    pub filters: Option<Filters>,
    pub constellations: Option<UpConstellations>,
}

/// Block offset of the raw bit `bit` of block `block` of a framed sequence.
fn raw(block: usize, bit: usize) -> usize {
    crate::v90::sequences::SYNC_ONES + block * (BLOCK + 1) + 1 + bit
}

impl Cpd {
    /// The CPd from its frame sync through CRC and one fill bit, or None if
    /// Table 30 forbids it: drn past 19, a filter longer than 9 bits can
    /// say, an empty, unsorted, zero-holding or over-128 set, more than 6
    /// sets, or an interval naming a set that is not there.
    pub fn to_bits(&self) -> Option<Vec<bool>> {
        if self.drn > UPSTREAM_RATES {
            return None;
        }
        let mut information = Vec::new();
        information.push(false); // 18: CPd
        information.push(self.moduli.is_some());
        information.push(self.filters.is_some());
        information.push(self.constellations.is_some());
        put(&mut information, u32::from(self.drn), 5);
        put(&mut information, self.trellis as u32, 2);
        information.push(self.extend_e2u);
        put(&mut information, 0, 3); // 30:32 reserved
        information.push(self.acknowledge);
        put(&mut information, u32::from(self.gain4), 16);
        if let Some(moduli) = self.moduli {
            for m in moduli {
                put(&mut information, u32::from(m), 8);
            }
        }
        if let Some(filters) = &self.filters {
            if filters.lengths().iter().any(|&l| l >= 512) {
                return None;
            }
            for l in filters.lengths() {
                put(&mut information, l as u32, 9);
                put(&mut information, 0, 7);
            }
            for c in filters.z1.iter().chain(&filters.p1).chain(&filters.z2).chain(&filters.p2) {
                put(&mut information, u32::from(*c as u16), 16);
            }
        }
        if let Some(c) = &self.constellations {
            let ok = (1..=6).contains(&c.sets.len())
                && c.sets.iter().all(|set| {
                    (1..=128).contains(&set.len()) && set[0] > 0 && set.windows(2).all(|w| w[0] < w[1])
                })
                && c.intervals.iter().all(|&i| usize::from(i) < c.sets.len());
            if !ok {
                return None;
            }
            for &i in &c.intervals[..4] {
                put(&mut information, u32::from(i), 4);
            }
            for &i in &c.intervals[4..] {
                put(&mut information, u32::from(i), 4);
            }
            put(&mut information, 0, 8); // reserved
            for k in 0..6 {
                put(&mut information, c.sets.get(k).map_or(0, |set| set.len() as u32), 8);
            }
            for point in c.sets.iter().flatten() {
                put(&mut information, u32::from(*point), 16);
            }
        }
        Some(with_fill(&information))
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let at = |i: usize| bits.get(i).copied();
        if at(18)? {
            return None;
        }
        let (moduli, filters, constellations) = (at(19)?, at(20)?, at(21)?);
        let value = |block: usize, bit: usize, width: usize| -> Option<usize> {
            (raw(block, bit + width - 1) < bits.len()).then(|| get(bits, raw(block, bit), width) as usize)
        };
        let mut blocks = 2;
        if moduli {
            blocks += 6;
        }
        let mut lengths = [0usize; 4];
        if filters {
            for (k, l) in lengths.iter_mut().enumerate() {
                *l = value(blocks + k, 0, 9)?;
            }
            blocks += 4 + lengths.iter().sum::<usize>();
        }
        let mut lc = [0usize; 6];
        if constellations {
            for (k, l) in lc.iter_mut().enumerate() {
                *l = value(blocks + 2 + k / 2, 8 * (k % 2), 8)?;
            }
            blocks += 5 + lc.iter().sum::<usize>();
        }
        let information = unframe(bits, blocks)?;
        let word = |block: usize| get(&information, block * BLOCK, 16) as u16;
        let mut block = 2;
        let moduli = moduli.then(|| {
            let m = std::array::from_fn(|k| get(&information, block * BLOCK + 8 * k, 8) as u8);
            block += 6;
            m
        });
        let filters = filters.then(|| {
            block += 4;
            let mut take = |n: usize| -> Vec<i16> {
                let v = (block..block + n).map(|b| word(b) as i16).collect();
                block += n;
                v
            };
            Filters { z1: take(lengths[0]), p1: take(lengths[1]), z2: take(lengths[2]), p2: take(lengths[3]) }
        });
        let constellations = if constellations {
            let intervals = std::array::from_fn(|i| {
                let bit = if i < 4 { block * BLOCK + 4 * i } else { (block + 1) * BLOCK + 4 * (i - 4) };
                get(&information, bit, 4) as u8
            });
            block += 5;
            let mut sets = Vec::new();
            for &n in lc.iter().take_while(|&&n| n > 0) {
                sets.push((block..block + n).map(word).collect());
                block += n;
            }
            Some(UpConstellations { intervals, sets })
        } else {
            None
        };
        Some(Self {
            drn: get(&information, 4, 5) as u8,
            trellis: match get(&information, 9, 2) {
                0 => UpTrellis::States16,
                1 => UpTrellis::States32,
                2 => UpTrellis::States64,
                _ => return None,
            },
            extend_e2u: information[11],
            acknowledge: information[15],
            gain4: word(1),
            moduli,
            filters,
            constellations,
        })
    }

    /// Whether the filters keep to what the analogue modem said it can run
    /// in INFO1a (Table 18): Ltot, Lmax and the sections it supports.
    pub fn fits(&self, request: &crate::v92::PcmUpstreamRequest) -> bool {
        let Some(filters) = &self.filters else { return true };
        let [z1, p1, z2, p2] = filters.lengths();
        let sections = request.filter_sections;
        z1 + p1 + z2 + p2 <= usize::from(request.total_coefficient_limit)
            && [z1, p1, z2, p2].iter().all(|&l| l <= usize::from(request.section_coefficient_limit))
            && (z1 == 0 || sections & 1 == 1)
            && (p2 == 0 || sections & 2 == 2)
    }
}

/// RM (Table 25) or, with `prime`, RM' (Table 26): the modulus encoder's
/// outputs K0 to K11 over one data frame, from the data mode moduli.
pub fn rm(moduli: &[u8; 12], prime: bool) -> [u16; 12] {
    std::array::from_fn(|i| {
        // RM takes the largest K in intervals 0, 1, 4, 5, 8, 9; RM' in the rest.
        if (i % 4 < 2) != prime { u16::from(moduli[i]).saturating_sub(1) } else { 0 }
    })
}

/// TRN2u's level for scrambler bits `bits`, MSB:LSB as Tables 28 and 29
/// write them, as a multiple of LU: the MSB is the sign.
pub fn trn2u_level(bits: u8, eight_point: bool) -> f64 {
    let (magnitude_bits, scale) = if eight_point { (2, 21f64.sqrt()) } else { (1, 5f64.sqrt()) };
    let magnitude = f64::from(2 * (bits & ((1 << magnitude_bits) - 1)) + 1) / scale;
    if bits >> magnitude_bits & 1 == 1 { -magnitude } else { magnitude }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test] fn upstream_drn_runs_24000_to_48000() {
        assert_eq!(upstream_rate(0), None);
        assert_eq!(upstream_rate(1), Some(24_000));
        assert_eq!(upstream_rate(13), Some(40_000));
        assert_eq!(upstream_rate(19), Some(48_000));
        assert_eq!(upstream_rate(20), None);
    }

    #[test] fn downstream_drn_matches_v90() {
        assert_eq!(downstream_rate(1), Some(28_000));
        assert_eq!(downstream_rate(22), Some(56_000));
        assert_eq!(downstream_rate(23), None);
    }

    pub(super) fn bits_value(b: &[bool], at: usize, width: usize) -> u32 {
        (0..width).fold(0, |v, i| v | u32::from(b[at + i]) << i)
    }

    #[test] fn cpu_header_follows_table_23() {
        let cp = UpCp { kind: CpKind::Data, drn: 22, acknowledge: true, a_law: true, lookahead: 2, ..UpCp::default() };
        let b = cp.to_bits();
        assert!(!b[18]);
        assert!(b[19] && !b[20]);
        assert_eq!(bits_value(&b, 21, 5), 22);
        assert!(b[26..31].iter().all(|&x| !x));
        assert!(b[33]);
        assert!(b[35]);
        assert!(b[36..49].iter().all(|&x| !x));
        assert_eq!(bits_value(&b, 49, 2), 2);
        assert_eq!(bits_value(&b, 52, 16), 1 << 13);
        assert_eq!(UpInfo::from_bits(&b), Some(UpInfo::Cp(cp)));
    }

    #[test] fn cpt_is_type_zero_with_its_masks() {
        let cp = UpCp { kind: CpKind::Training, drn: 4, intervals: [0, 1, 0, 1, 0, 1],
                        constellations: vec![1 << 79 | 1, 1 << 127], ..UpCp::default() };
        let b = cp.to_bits();
        assert!(!b[19] && !b[20]);
        assert!(b[137] && b[137 + 16 * 4 + 4 + 15]);  // Ucode 0; Ucode 79 is chord 5 bit 15
        assert!(b[256 + 136 + 15]);                    // second set, Ucode 127
        assert_eq!(UpInfo::from_bits(&b), Some(UpInfo::Cp(cp)));
    }

    #[test] fn cpus_is_two_blocks_of_header_and_a_crc() {
        let b = Cpus { drn: 5, acknowledge: false }.to_bits();
        assert_eq!(b.len(), 52);
        assert!(!b[18] && !b[19] && b[20]);
        assert_eq!(bits_value(&b, 21, 5), 5);
        assert!(!b[34] && !b[51]);
        assert_eq!(UpInfo::from_bits(&b), Some(UpInfo::Cpus(Cpus { drn: 5, acknowledge: false })));
    }

    #[test] fn suvu_level_and_flags() {
        let s = Suvu { wait_for_cpu: true, level_q2_2: -16, silence: true, acknowledge: false };
        let b = s.to_bits();
        assert_eq!(b.len(), 52);
        assert!(b[18] && b[26] && b[32] && !b[33]);
        assert!(b[19..26].iter().all(|&x| !x));
        assert_eq!(bits_value(&b, 27, 5), 16);
        assert_eq!(UpInfo::from_bits(&b), Some(UpInfo::Suv(s)));
        let up = Suvu { level_q2_2: 7, ..s }.to_bits();
        assert_eq!(bits_value(&up, 27, 5), 7);
    }

    #[test] fn suvd_is_suv_with_reserved_19_to_31() {
        let s = Suvd { silence: false, acknowledge: true };
        let b = s.to_bits();
        assert!(b[18] && b[19..32].iter().all(|&x| !x) && !b[32] && b[33]);
        assert_eq!(Suvd::from_bits(&b), Some(s));
    }

    #[test] fn pad_fills_with_zeros_to_the_multiple() {
        let mut b = vec![true; 52];
        pad(&mut b, 24);
        assert_eq!(b.len(), 72);
        assert!(b[52..].iter().all(|&x| !x));
        pad(&mut b, 24);
        assert_eq!(b.len(), 72);
    }

    fn cpd_base() -> Cpd {
        Cpd { drn: 19, trellis: UpTrellis::States64, extend_e2u: false, acknowledge: false, gain4: 0x8000,
              moduli: None, filters: None, constellations: None }
    }

    #[test] fn the_always_present_part_is_bits_0_to_50() {
        let b = Cpd { acknowledge: true, ..cpd_base() }.to_bits().unwrap();
        assert!(b[0..17].iter().all(|&x| x));
        assert!(!b[17] && !b[18] && !b[19] && !b[20] && !b[21]);
        assert_eq!(bits_value(&b, 22, 5), 19);
        assert_eq!(bits_value(&b, 27, 2), 2);
        assert!(!b[29] && b[30..33].iter().all(|&x| !x));
        assert!(b[33] && !b[34] && b[50] && !b[35]);
        assert!(!b[51]);
        assert_eq!(b.len(), 69);
        assert!(!b[68]);
        assert_eq!(Cpd::from_bits(&b), Some(Cpd { acknowledge: true, ..cpd_base() }));
    }

    #[test] fn moduli_sit_at_52_to_152_when_present() {
        let m = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 128];
        let cpd = Cpd { moduli: Some(m), ..cpd_base() };
        let b = cpd.to_bits().unwrap();
        assert!(b[19]);
        assert_eq!(bits_value(&b, 52, 8), 1);
        assert_eq!(bits_value(&b, 60, 8), 2);
        assert!(!b[68]);
        assert_eq!(bits_value(&b, 69, 8), 3);
        assert_eq!(bits_value(&b, 137, 8), 11);
        assert_eq!(bits_value(&b, 145, 8), 128);
        assert!(!b[153]);
        assert_eq!(Cpd::from_bits(&b), Some(cpd));
    }

    #[test] fn absent_moduli_shift_the_filters_back_six_blocks() {
        let f = Filters { z1: vec![], p1: vec![0x4000], z2: vec![0x7fff, -1], p2: vec![] };
        let b = Cpd { filters: Some(f.clone()), ..cpd_base() }.to_bits().unwrap();
        assert!(!b[19] && b[20]);
        assert_eq!(bits_value(&b, 52, 9), 0);
        assert_eq!(bits_value(&b, 69, 9), 1);
        assert_eq!(bits_value(&b, 86, 9), 2);
        assert_eq!(bits_value(&b, 103, 9), 0);
        assert!(!b[119]);
        assert_eq!(bits_value(&b, 120, 16) as u16, 0x4000);
        assert_eq!(bits_value(&b, 137, 16) as u16, 0x7fff);
        assert_eq!(bits_value(&b, 154, 16) as u16, 0xffff);
        assert_eq!(Cpd::from_bits(&b).unwrap().filters, Some(f));
    }

    #[test] fn constellations_follow_alpha() {
        let f = Filters { z1: vec![1], p1: vec![2], z2: vec![3], p2: vec![4] };
        let c = UpConstellations { intervals: [0, 1, 0, 1, 0, 1], sets: vec![vec![8, 100, 4000], vec![16]] };
        let cpd = Cpd { moduli: Some([2; 12]), filters: Some(f), constellations: Some(c), ..cpd_base() };
        let b = cpd.to_bits().unwrap();
        let alpha = 68;
        assert!(b[19] && b[20] && b[21]);
        assert_eq!(bits_value(&b, 222, 16), 1);                    // z1(1)
        assert_eq!(bits_value(&b, 273, 16), 4);                    // p2(1)
        assert!(!b[221 + alpha]);
        assert_eq!(bits_value(&b, 222 + alpha, 4), 0);
        assert_eq!(bits_value(&b, 226 + alpha, 4), 1);
        assert_eq!(bits_value(&b, 243 + alpha, 4), 1);              // interval 5
        assert_eq!(bits_value(&b, 247 + alpha, 8), 0);
        assert_eq!(bits_value(&b, 256 + alpha, 8), 3);              // LC1
        assert_eq!(bits_value(&b, 264 + alpha, 8), 1);              // LC2
        assert_eq!(bits_value(&b, 273 + alpha, 8), 0);              // LC3
        assert_eq!(bits_value(&b, 307 + alpha, 16), 8);
        assert_eq!(bits_value(&b, 324 + alpha, 16), 100);
        assert_eq!(bits_value(&b, 358 + alpha, 16), 16);            // LC2's point
        let beta = 17 * 4;
        assert!(!b[306 + alpha + beta]);
        assert_eq!(b.len(), 324 + alpha + beta);                    // CRC then one fill bit
        assert_eq!(Cpd::from_bits(&b), Some(cpd));
    }

    #[test] fn refuses_what_table_30_forbids() {
        let with = |sets: Vec<Vec<u16>>, intervals: [u8; 6]| Cpd {
            constellations: Some(UpConstellations { intervals, sets }), ..cpd_base() }.to_bits();
        assert!(with(vec![vec![0, 5]], [0; 6]).is_none(), "zero point");
        assert!(with(vec![(1..=129).collect()], [0; 6]).is_none(), "129 points");
        assert!(with(vec![(1..=128).collect()], [0; 6]).is_some());
        assert!(with(vec![vec![5, 3]], [0; 6]).is_none(), "not smallest first");
        assert!(with(vec![vec![5], vec![]], [0; 6]).is_none(), "an empty set");
        assert!(with(vec![vec![5]], [0, 1, 0, 0, 0, 0]).is_none(), "interval past the last set");
        assert!(with(vec![vec![1]; 7], [0; 6]).is_none(), "seven sets");
        assert!(Cpd { drn: 20, ..cpd_base() }.to_bits().is_none());
        assert!(Cpd { drn: 0, ..cpd_base() }.to_bits().is_some(), "cleardown");
        let long = Filters { z1: vec![], p1: vec![0; 512], z2: vec![], p2: vec![] };
        assert!(Cpd { filters: Some(long), ..cpd_base() }.to_bits().is_none());
    }

    #[test] fn fits_checks_ltot_lmax_and_sections() {
        let request = crate::v92::PcmUpstreamRequest { filter_sections: 0, total_coefficient_limit: 192,
            section_coefficient_limit: 128, md_units: 0, uinfo: 80 };
        let with = |z1: usize, p1: usize, z2: usize, p2: usize| Cpd {
            filters: Some(Filters { z1: vec![1; z1], p1: vec![1; p1], z2: vec![1; z2], p2: vec![1; p2] }),
            ..cpd_base() }.fits(&request);
        assert!(with(0, 64, 128, 0));
        assert!(!with(1, 64, 64, 0), "z1 needs sections 1 or 3");
        assert!(!with(0, 0, 64, 1), "p2 needs sections 2 or 3");
        assert!(!with(0, 0, 129, 0), "past Lmax");
        assert!(!with(0, 100, 100, 0), "past Ltot");
        let all = crate::v92::PcmUpstreamRequest { filter_sections: 3, ..request };
        assert!(Cpd { filters: Some(Filters { z1: vec![1; 40], p1: vec![1; 40], z2: vec![1; 40], p2: vec![1; 40] }),
            ..cpd_base() }.fits(&all));
        assert!(cpd_base().fits(&request), "no filters sent");
    }

    #[test] fn a_damaged_cpd_is_refused() {
        let mut b = Cpd { moduli: Some([9; 12]), ..cpd_base() }.to_bits().unwrap();
        b[60] ^= true;
        assert_eq!(Cpd::from_bits(&b), None);
    }

    #[test] fn rm_and_rm_prime_follow_tables_25_and_26() {
        let m = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21];
        assert_eq!(rm(&m, false), [9, 10, 0, 0, 13, 14, 0, 0, 17, 18, 0, 0]);
        assert_eq!(rm(&m, true), [0, 0, 11, 12, 0, 0, 15, 16, 0, 0, 19, 20]);
    }

    #[test] fn trn2u_maps_follow_tables_28_and_29() {
        let r5 = 5f64.sqrt();
        let r21 = 21f64.sqrt();
        assert_eq!(trn2u_level(0b00, false), 1. / r5);
        assert_eq!(trn2u_level(0b01, false), 3. / r5);
        assert_eq!(trn2u_level(0b10, false), -1. / r5);
        assert_eq!(trn2u_level(0b11, false), -3. / r5);
        assert_eq!(trn2u_level(0b011, true), 7. / r21);
        assert_eq!(trn2u_level(0b100, true), -1. / r21);
        assert_eq!(trn2u_level(0b111, true), -7. / r21);
    }
}
