//! V.92 Phase 3 with PCM upstream, the analogue modem's side as the digital
//! modem hears it (Figure 10): Ru, R̄u, TRN1u, Ja; then Su with its half-symbol
//! S̄u (9.5.2.1.7), the S̄u that answers Jp extended by Jp's fraction
//! (9.5.2.1.8), the second TRN1u and CPt. The analogue modem's symbols are
//! sent at fractional times through a continuous telephone-band response,
//! so the half symbol really moves its sampling.

use datapump::v90::sequences::Descriptor;
use datapump::v90::ucode::{self, Law};
use datapump::v92::phase3::JaDescriptor;
use datapump::v92::phase4::{CpKind, UpCp, UpInfo};
use datapump::v92::upstream::training::{ru, su, Heard, Listener, Sender, SuTurn};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn gaussian(&mut self) -> f64 {
        let u = ((self.next() >> 11) as f64 / (1u64 << 53) as f64).max(1e-300);
        let v = (self.next() >> 11) as f64 / (1u64 << 53) as f64;
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// 300-3400 Hz band-pass at time `t` symbols after a symbol is sent: the
/// 31-tap windowed sinc of the other tests, at any t in [0, 30].
fn band(t: f64) -> f64 {
    if !(0.0..=30.0).contains(&t) {
        return 0.0;
    }
    let fs = 8000.0;
    let m = t - 15.0;
    let sinc = |x: f64| if x.abs() < 1e-12 { 1.0 } else { (std::f64::consts::PI * x).sin() / (std::f64::consts::PI * x) };
    let hamming = 0.54 - 0.46 * (std::f64::consts::TAU * t / 30.0).cos();
    (2.0 * 3400.0 / fs * sinc(2.0 * 3400.0 * m / fs) - 2.0 * 300.0 / fs * sinc(2.0 * 300.0 * m / fs)) * hamming
}

/// Symbols at times, to codec octets.
fn through(symbols: &[(f64, f64)], law: Law, sigma: f64) -> Vec<u8> {
    let end = symbols.last().map_or(0.0, |s| s.0).ceil() as usize + 40;
    let mut y = vec![0.0; end];
    for &(at, v) in symbols {
        if v == 0.0 {
            continue;
        }
        let first = at.ceil() as usize;
        for (n, out) in y.iter_mut().enumerate().skip(first).take(32) {
            *out += v * band(n as f64 - at);
        }
    }
    let mut rng = Rng(9);
    y.iter()
        .map(|&v| {
            let (u, negative) = ucode::nearest(law, (v * 2800.0 + sigma * rng.gaussian()).round() as i32);
            ucode::octet(law, u, negative)
        })
        .collect()
}

/// The analogue modem's transmitter clock: symbols one apart from `at`.
struct Clock {
    at: f64,
    out: Vec<(f64, f64)>,
}

impl Clock {
    fn send(&mut self, values: impl IntoIterator<Item = f64>) {
        for v in values {
            self.out.push((self.at, v));
            self.at += 1.0;
        }
    }
    fn wait(&mut self, symbols: f64) {
        self.at += symbols;
    }
}

fn ja() -> JaDescriptor {
    JaDescriptor { dil: Descriptor::none(), upstream_rates: 0x7_ffff }
}

fn cpt() -> UpCp {
    UpCp { kind: CpKind::Training, drn: 9, ..UpCp::default() }
}

/// Figure 10's upstream, Jp asking for `extension` of a symbol. Returns the
/// symbols and where (in samples) the analogue modem would have heard Jp.
fn phase3(extension: f64, gap: f64) -> Vec<(f64, f64)> {
    phase3_from(0.0, 0.0, extension, gap)
}

/// [`phase3`] with the first TRN1u `first` of a symbol off the codec's
/// samples, and the analogue modem's clock moved by `moved` of a symbol
/// while it trains on TRN1d, as locking to the network's clock moves it.
fn phase3_from(first: f64, moved: f64, extension: f64, gap: f64) -> Vec<(f64, f64)> {
    phase3_full(first, moved, extension, gap, false)
}

/// [`phase3_from`], the extension taken either after the 24T of S-bar-u or,
/// with `early`, before them: the Su before is stretched and S-bar-u arrives
/// already moved. A Conexant V.92 modem does the second (live, 2026-11-11).
fn phase3_full(first: f64, moved: f64, extension: f64, gap: f64, early: bool) -> Vec<(f64, f64)> {
    let mut s = Sender::new();
    let mut c = Clock { at: 0.0, out: Vec::new() };
    c.wait(400.0 + first);
    c.send(ru(384, false));
    c.send(ru(24, true));
    c.send(s.trn1u(2400));
    let mut bits = vec![true; 24];
    bits.extend(ja().to_bits());
    bits.extend(ja().to_bits());
    c.send(s.two_point(&bits));
    // Silence while the digital modem sends Sd, TRN1d and Jd.
    c.wait(3000.0 + moved);
    // 9.5.2.1.6-7: Su 144T, S̄u 24.5T, then Su until Jp.
    c.send(su(144, false));
    c.send(su(24, true));
    c.wait(0.5);
    c.send(su(600, false));
    // 9.5.2.1.8: Jp heard; S̄u for 24T plus the fraction, then TRN1u.
    if early {
        c.wait(extension);
    }
    c.send(su(24, true));
    if !early {
        c.wait(extension);
    }
    // 9.5.2.1.9: TRN1u only once Jp' is detected; silence until then.
    c.wait(gap);
    c.send(s.trn1u(2400));
    let mut bits = vec![true; 24];
    for _ in 0..3 {
        bits.extend(cpt().to_bits());
    }
    bits.extend([false; 12]); // E1u
    c.send(s.two_point(&bits));
    c.wait(200.0);
    c.out
}

fn find<T>(bits: &[bool], parse: impl Fn(&[bool]) -> Option<T>) -> Option<T> {
    (0..bits.len()).find_map(|i| parse(&bits[i..]))
}

struct Heard3 {
    ja: bool,
    turns: Vec<SuTurn>,
    trained: Vec<f64>,
    cpt: Option<UpInfo>,
}

/// The digital modem's side, as 9.5.1.1 orders it: train, read Ja, watch
/// Su, retrain on the second TRN1u after the second Su-to-S̄u, read CPt.
fn listen(extension: f64, law: Law) -> Heard3 {
    listen_gap(extension, 0.0, law)
}

fn listen_gap(extension: f64, gap: f64, law: Law) -> Heard3 {
    listen_octets(through(&phase3(extension, gap), law, 8.0), law)
}

fn listen_octets(octets: Vec<u8>, law: Law) -> Heard3 {
    let mut l = Listener::new(law, 0);
    let mut bits = Vec::new();
    let (mut ja, mut watching, mut second) = (false, false, false);
    let mut turns = Vec::new();
    let mut trained = Vec::new();
    for o in octets {
        if let Some(Heard::Trained { snr_db }) = l.push(o) {
            trained.push(snr_db);
        }
        if !ja {
            bits.extend(l.take_bits());
            if find(&bits, JaDescriptor::from_bits).is_some() {
                ja = true;
            }
        } else if !watching {
            l.listen_su();
            watching = true;
        } else if !second {
            for t in l.take_su_turns() {
                turns.push(t);
                let to_bar: Vec<usize> = turns.iter().filter_map(|t| match t { SuTurn::ToBar(n) => Some(*n), _ => None }).collect();
                if to_bar.len() == 2 {
                    // TRN1u follows the 24T-and-a-fraction S̄u.
                    l.expect_trn1u(to_bar[1] + 24);
                    second = true;
                    bits.clear();
                }
            }
        } else {
            bits.extend(l.take_bits());
        }
    }
    let cpt = find(&bits, |b| UpInfo::from_bits(b).filter(|i| matches!(i, UpInfo::Cp(c) if c.kind == CpKind::Training)));
    Heard3 { ja, turns, trained, cpt }
}

#[test]
fn su_s_two_transitions_are_seen_and_cpt_reads_after_the_second_trn1u() {
    let h = listen(0.5, Law::Mu);
    assert!(h.ja, "Ja");
    assert!(matches!(h.turns.as_slice(), [SuTurn::ToBar(_), SuTurn::ToSu(_), SuTurn::ToBar(_), ..]), "{:?}", h.turns);
    eprintln!("turns {:?}, trained {:?}", h.turns, h.trained);
    assert_eq!(h.trained.len(), 2, "{:?}", h.trained);
    assert!(h.trained[1] > 20.0, "{:?}", h.trained);
    assert_eq!(h.cpt, Some(UpInfo::Cp(cpt())));
}

#[test]
fn any_phase_jp_leaves_is_trained_out_on_the_second_trn1u() {
    for extension in [0.0, 0.25, 0.75] {
        let h = listen(extension, Law::A);
        eprintln!("extension {extension}: trained {:?}", h.trained);
        assert_eq!(h.cpt, Some(UpInfo::Cp(cpt())), "extension {extension}");
    }
}

/// Figure 10: the second TRN1u comes when the analogue modem has detected
/// Jp', not straight after its S-bar-u; how long that takes is its own.
#[test]
fn the_second_trn1u_is_found_after_a_silence_of_any_length() {
    for gap in [137.0, 400.3, 36.0] {
        let h = listen_gap(0.5, gap, Law::Mu);
        eprintln!("gap {gap}: trained {:?}", h.trained);
        assert_eq!(h.trained.len(), 2, "gap {gap}");
        assert!(h.trained[1] > 28.0, "gap {gap}: {:?}", h.trained);
        assert_eq!(h.cpt, Some(UpInfo::Cp(cpt())), "gap {gap}");
    }
}

fn peak(h: &[f64]) -> f64 {
    h.iter().fold(0.0f64, |m, v| m.max(v.abs()))
}

/// What 9.5.1.1.6-8 has the digital modem do: measure Su once it is back
/// from S-bar-u and say what extension Jp should ask for. Then the second
/// TRN1u's main tap with that extension, and with `instead` if given.
fn steered(first: f64, moved: f64, instead: Option<f64>) -> (f64, f64) {
    let law = Law::Mu;
    let mut l = Listener::new(law, 0);
    let (mut ja, mut watching) = (false, false);
    let mut bits = Vec::new();
    let mut back: Option<usize> = None;
    let mut asked = None;
    for o in through(&phase3_from(first, moved, 0.0, 0.0), law, 8.0) {
        l.push(o);
        if !ja {
            bits.extend(l.take_bits());
            ja = find(&bits, JaDescriptor::from_bits).is_some();
        } else if !watching {
            l.listen_su();
            watching = true;
        } else if back.is_none() {
            if l.take_su_turns().iter().any(|t| matches!(t, SuTurn::ToSu(_))) {
                back = Some(l.received());
            }
        } else if l.received() == back.unwrap() + 192 {
            asked = Some((l.best_phase().expect("a channel") - l.su_phase(96).expect("Su")).rem_euclid(1.0));
            break;
        }
    }
    let asked = asked.expect("Su came back from S-bar-u");
    // The call again, the analogue modem doing as Jp asked.
    let mut l = Listener::new(law, 0);
    let (mut ja, mut watching, mut second) = (false, false, false);
    let mut bits = Vec::new();
    let mut to_bar = Vec::new();
    let mut tap = 0.0;
    for o in through(&phase3_from(first, moved, instead.unwrap_or(asked), 0.0), law, 8.0) {
        if let Some(Heard::Trained { .. }) = l.push(o) {
            tap = peak(&l.channel().expect("trained").h);
        }
        if !ja {
            bits.extend(l.take_bits());
            ja = find(&bits, JaDescriptor::from_bits).is_some();
        } else if !watching {
            l.listen_su();
            watching = true;
        } else if !second {
            to_bar.extend(l.take_su_turns().into_iter().filter_map(|t| match t {
                SuTurn::ToBar(n) => Some(n),
                _ => None,
            }));
            if to_bar.len() == 2 {
                l.expect_trn1u(to_bar[1] + 24);
                second = true;
            }
        }
    }
    assert!(second, "the second TRN1u was looked for");
    (asked, tap)
}

/// 9.5.1.1.6-8 and Table 22: the digital modem cannot move the codec's
/// sampling, so it measures Su and has Jp move the analogue modem's
/// transmitter. Wherever the modem's clock was left, the second TRN1u then
/// reaches the codec as it does at the best phase.
#[test]
fn jp_s_extension_puts_the_second_trn1u_on_the_codec_s_samples() {
    // The best there is on this path: symbols sent on the samples.
    let (_, on_the_samples) = steered(0.0, 0.5, None);
    for (first, moved) in [(0.0, 0.0), (0.0, 0.3), (0.3, 0.0), (0.6, 0.85), (0.45, 0.5), (0.9, 0.1), (0.0, 0.55), (0.0, 0.52), (0.0, 0.49), (0.2, 0.33)] {
        let (asked, tap) = steered(first, moved, None);
        eprintln!("first {first}, moved {moved}: Jp asks {asked:.3}, main tap {tap:.4} of {on_the_samples:.4}");
        assert!(tap > 0.99 * on_the_samples, "first {first}, moved {moved}: asked {asked:.3}, tap {tap} of {on_the_samples}");
    }
    // And it is the extension that does it: half a symbol off, the tap is
    // well down.
    let (asked, _) = steered(0.0, 0.3, None);
    let (_, off) = steered(0.0, 0.3, Some((asked + 0.5).rem_euclid(1.0)));
    assert!(off < 0.9 * on_the_samples, "half a symbol off still gave {off} of {on_the_samples}");
}

/// The S-bar-u that answers Jp carries Jp's extension, and a modem may put
/// it first: S-bar-u then arrives turned by half a turn less up to a sixth,
/// not by half a turn. It is still the transition 9.5.1.1.9 waits for; taken
/// for nothing, the second TRN1u goes by unheard (live: every call that
/// asked for more than three quarters of a symbol).
#[test]
fn the_s_bar_u_answering_jp_is_seen_with_its_extension_first() {
    for extension in [0.1, 0.5, 0.74, 0.77, 0.84, 0.97] {
        let h = listen_octets(through(&phase3_full(0.0, 0.0, extension, 0.0, true), Law::Mu, 8.0), Law::Mu);
        eprintln!("extension {extension}: turns {:?}, trained {:?}", h.turns, h.trained);
        assert_eq!(h.trained.len(), 2, "extension {extension}: {:?}", h.turns);
        assert_eq!(h.cpt, Some(UpInfo::Cp(cpt())), "extension {extension}");
    }
}
