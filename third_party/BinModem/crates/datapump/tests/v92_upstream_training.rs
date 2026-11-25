//! V.92 PCM upstream training signals through a telephone-band path: the
//! analogue modem's Ru, R̄u, TRN1u, Ja, CPt, E1u, TRN2u, SUVu and CPu, and
//! the digital modem's Listener reading them back.

use datapump::v90::sequences::Descriptor;
use datapump::v90::ucode::{self, Law};
use datapump::v92::phase3::JaDescriptor;
use datapump::v92::phase4::{CpKind, Suvu, UpCp, UpInfo};
use datapump::v92::upstream::training::{ru, Heard, Listener, Sender};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn uniform(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn gaussian(&mut self) -> f64 {
        let (u, v) = (self.uniform().max(1e-300), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// 300-3400 Hz at 8 kHz: 31-tap windowed-sinc band-pass, unity in band.
fn telephone_band() -> Vec<f64> {
    let fs = 8000.0;
    let sinc = |x: f64| if x == 0.0 { 1.0 } else { (std::f64::consts::PI * x).sin() / (std::f64::consts::PI * x) };
    (0..31)
        .map(|n| {
            let m = n as f64 - 15.0;
            let hamming = 0.54 - 0.46 * (std::f64::consts::TAU * n as f64 / 30.0).cos();
            (2.0 * 3400.0 / fs * sinc(2.0 * 3400.0 * m / fs) - 2.0 * 300.0 / fs * sinc(2.0 * 300.0 * m / fs)) * hamming
        })
        .collect()
}

/// Transmit units (LU = 1) to codec octets: the band, 2800 codec units per
/// unit times `gain` (negative inverts), noise, the codec.
fn through(tx: &[f64], law: Law, gain: f64, sigma: f64, seed: u64, impulse: Option<usize>) -> Vec<u8> {
    let fir = telephone_band();
    let mut history = vec![0.0; fir.len()];
    let mut rng = Rng(seed);
    tx.iter()
        .enumerate()
        .map(|(n, &t)| {
            history.rotate_right(1);
            history[0] = t;
            let y = fir.iter().zip(&history).map(|(h, x)| h * x).sum::<f64>() * 2800.0 * gain + sigma * rng.gaussian()
                + if impulse == Some(n) { 8000.0 } else { 0.0 };
            let (u, negative) = ucode::nearest(law, y.round() as i32);
            ucode::octet(law, u, negative)
        })
        .collect()
}

fn ja() -> JaDescriptor {
    JaDescriptor { dil: Descriptor::none(), upstream_rates: 0x7_ffff }
}

fn cpt() -> UpCp {
    UpCp { kind: CpKind::Training, drn: 9, ..UpCp::default() }
}

fn suvu() -> Suvu {
    Suvu { wait_for_cpu: false, level_q2_2: -16, silence: false, acknowledge: false }
}

fn cpu() -> UpCp {
    UpCp { kind: CpKind::Data, drn: 16, acknowledge: true, ..UpCp::default() }
}

/// The analogue modem's Phase 3 and 4 upstream, from silence to E2u; the
/// symbol, counted from TRN1u's first, where TRN2u begins; and the sample
/// where the CPts begin.
fn sequence(eight: bool, md: usize) -> (Vec<f64>, usize, usize) {
    let mut s = Sender::new();
    let mut tx = vec![0.0; 400];
    tx.extend(ru(384, false));
    tx.extend(ru(24, true));
    if md > 0 {
        // MD is the manufacturer's (V.34 10.1.3.5): anything; here noise-like.
        let mut rng = Rng(77);
        tx.extend((0..md).map(|_| if rng.next() & 1 == 1 { 1.0 } else { -1.0 }));
        tx.extend(ru(384, false));
        tx.extend(ru(24, true));
    }
    let trn1u_at = tx.len();
    tx.extend(s.trn1u(2400));
    let mut bits = vec![true; 24];
    bits.extend(ja().to_bits());
    bits.extend(ja().to_bits());
    tx.extend(s.two_point(&bits));
    tx.extend(s.trn1u(2040));
    let cpt_at = tx.len();
    let mut bits = vec![true; 24];
    for _ in 0..3 {
        bits.extend(cpt().to_bits());
    }
    tx.extend(s.two_point(&bits));
    tx.extend(s.two_point(&[false; 12])); // E1u
    let trn2u_at = tx.len() - trn1u_at;
    tx.extend(s.trn2u(2400, eight));
    let mut bits = Vec::new();
    bits.extend(suvu().to_bits());
    bits.extend(suvu().to_bits());
    bits.extend(cpu().to_bits());
    bits.extend([false; 36]); // E2u, a data frame
    tx.extend(s.trn2u_bits(&bits, eight));
    tx.extend(vec![0.0; 200]);
    (tx, trn2u_at, cpt_at)
}

fn find<T>(bits: &[bool], parse: impl Fn(&[bool]) -> Option<T>) -> Option<T> {
    (0..bits.len()).find_map(|i| parse(&bits[i..]))
}

fn find_info(bits: &[bool], want: impl Fn(&UpInfo) -> bool) -> Option<UpInfo> {
    find(bits, |b| UpInfo::from_bits(b).filter(|i| want(i)))
}

struct Read {
    heard: Vec<Heard>,
    ja: Option<JaDescriptor>,
    cpt: Option<UpInfo>,
    suvu: Option<UpInfo>,
    cpu: Option<UpInfo>,
}

#[derive(Default)]
struct Case {
    md: usize,
    /// Symbols into the first CPt where a noise impulse hits.
    impulse: Option<usize>,
    /// Take nothing before the switch to TRN2u.
    late_take: bool,
}

fn listen(law: Law, gain: f64, sigma: f64, eight: bool, seed: u64) -> Read {
    listen_case(law, gain, sigma, eight, seed, Case::default())
}

fn listen_case(law: Law, gain: f64, sigma: f64, eight: bool, seed: u64, case: Case) -> Read {
    let (tx, trn2u_at, cpt_at) = sequence(eight, case.md);
    let octets = through(&tx, law, gain, sigma, seed, case.impulse.map(|i| cpt_at + i));
    let mut l = Listener::new(law, case.md);
    let mut heard = Vec::new();
    for o in octets {
        heard.extend(l.push(o));
    }
    let (two_point, trn2u) = if case.late_take {
        l.switch_trn2u(trn2u_at, eight);
        let all = l.take_bits();
        (all.clone(), all)
    } else {
        let two_point = l.take_bits();
        l.switch_trn2u(trn2u_at, eight);
        (two_point, l.take_bits())
    };
    Read {
        heard,
        ja: find(&two_point, JaDescriptor::from_bits),
        cpt: find_info(&two_point, |i| matches!(i, UpInfo::Cp(c) if c.kind == CpKind::Training)),
        suvu: find_info(&trn2u, |i| matches!(i, UpInfo::Suv(_))),
        cpu: find_info(&trn2u, |i| matches!(i, UpInfo::Cp(c) if c.kind == CpKind::Data)),
    }
}

fn assert_all_read(r: &Read, what: &str) {
    assert!(matches!(r.heard.first(), Some(Heard::RuReversal)), "{what}: {:?}", r.heard);
    let snr = r.heard.iter().find_map(|h| match h { Heard::Trained { snr_db } => Some(*snr_db), _ => None });
    eprintln!("{what}: trained at {snr:?} dB");
    assert!(snr.is_some_and(|s| s > 15.0), "{what}: {snr:?}");
    assert_eq!(r.ja.as_ref(), Some(&ja()), "{what}: Ja");
    assert_eq!(r.cpt, Some(UpInfo::Cp(cpt())), "{what}: CPt");
    assert_eq!(r.suvu, Some(UpInfo::Suv(suvu())), "{what}: SUVu");
    assert_eq!(r.cpu, Some(UpInfo::Cp(cpu())), "{what}: CPu");
}

#[test]
fn ja_and_cpt_cross_a_telephone_band() {
    let r = listen(Law::Mu, 1.0, 8.0, false, 1);
    assert_all_read(&r, "mu-law");
}

#[test]
fn a_law_inverted_and_mislevelled_lines_read_the_same() {
    for (law, gain) in [(Law::A, 1.0), (Law::Mu, -0.5), (Law::A, 2.0)] {
        let r = listen(law, gain, 8.0, false, 2);
        assert_all_read(&r, &format!("{law:?} gain {gain}"));
    }
}

#[test]
fn trn2u_carries_suvu_and_cpu_at_eight_points() {
    let r = listen(Law::Mu, 1.0, 8.0, true, 3);
    assert_all_read(&r, "8-point");
}

#[test]
fn noise_beyond_reading_gives_no_bits_not_wrong_ones() {
    let r = listen(Law::Mu, 1.0, 3000.0, false, 4);
    assert!(r.ja.is_none() || r.ja == Some(ja()));
    assert!(r.cpu.is_none() || r.cpu == Some(UpInfo::Cp(cpu())));
}

#[test]
fn md_between_the_two_rus_is_let_go_by() {
    let r = listen_case(Law::Mu, 1.0, 8.0, false, 5, Case { md: 280, ..Case::default() });
    assert_all_read(&r, "MD 280");
}

#[test]
fn bits_not_taken_before_the_switch_are_kept() {
    let r = listen_case(Law::Mu, 1.0, 8.0, false, 6, Case { late_take: true, ..Case::default() });
    assert_all_read(&r, "late take");
}

#[test]
fn an_impulse_in_cpt_spoils_no_more_than_the_cpt_it_hits() {
    // Decision feedback carries a wrong decision into the next 32 symbols;
    // the CPt after the hit one, and everything later, must still read.
    let r = listen_case(Law::Mu, 1.0, 8.0, false, 7, Case { impulse: Some(100), ..Case::default() });
    assert_all_read(&r, "impulse");
}
