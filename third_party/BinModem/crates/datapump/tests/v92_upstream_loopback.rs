//! V.92 PCM upstream end to end: our test modem's transmitter, an analogue
//! path, a G.711 codec, and the digital modem's receiver.

use datapump::v90::ucode::{self, Law};
use datapump::v92::phase4::{Cpd, Filters, UpConstellations, UpTrellis};
use datapump::v92::upstream::params::UpParams;
use datapump::v92::upstream::receiver::{Decoding, Receiver};
use datapump::v92::upstream::transmitter::Transmitter;

const GAIN4: u16 = 32768; // G = 0.125

/// A CPd with one set of mu-law levels (Ucodes `ucodes`) and `moduli`.
fn cpd(drn: u8, moduli: [u8; 12], ucodes: impl Iterator<Item = u8>, filters: Option<Filters>) -> Cpd {
    let points: Vec<u16> = ucodes.map(|u| ucode::linear(Law::Mu, u) as u16).collect();
    Cpd {
        drn,
        trellis: UpTrellis::States16,
        extend_e2u: false,
        acknowledge: false,
        gain4: GAIN4,
        moduli: Some(moduli),
        filters,
        constellations: Some(UpConstellations { intervals: [0; 6], sets: vec![points] }),
    }
}

/// drn 7, moduli 16, 32 levels: Ucodes 40, 42, ..., 102.
fn fixture(filters: Option<Filters>) -> UpParams {
    UpParams::from_cpd(&cpd(7, [16; 12], (40..=102).step_by(2), filters)).unwrap()
}

struct Rng(u64);
impl Rng {
    fn bit(&mut self) -> bool {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0 & 1 == 1
    }
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn gaussian(&mut self) -> f64 {
        let (u, v) = (self.uniform().max(1e-300), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

fn data(frames: usize, k: u32, seed: u64) -> Vec<Vec<bool>> {
    let mut rng = Rng(seed);
    (0..frames).map(|_| (0..k).map(|_| rng.bit()).collect()).collect()
}

/// The path from the transmitter's output to the codec: a gain, an FIR and
/// additive noise of standard deviation `sigma` (in codec linear units).
struct Path {
    gain: f64,
    fir: Vec<f64>,
    sigma: f64,
}

/// Send `frames` through `path` and a mu-law codec; the bits of each frame
/// as received.
fn run(params: &UpParams, path: &Path, decoding: Decoding, frames: &[Vec<bool>], seed: u64) -> Vec<Vec<bool>> {
    let mut tx = Transmitter::new(params.clone());
    let mut rx = Receiver::new(params.clone(), Law::Mu, decoding).unwrap();
    let mut rng = Rng(seed);
    let mut history = vec![0.0; path.fir.len()];
    let mut bits = Vec::new();
    for f in frames {
        for s in tx.frame(f).unwrap().samples {
            history.rotate_right(1);
            history[0] = s;
            let line: f64 = path.fir.iter().zip(&history).map(|(h, x)| h * x).sum::<f64>() * path.gain
                + path.sigma * rng.gaussian();
            let (u, negative) = ucode::nearest(Law::Mu, line.round() as i32);
            bits.extend(rx.push_octet(ucode::octet(Law::Mu, u, negative)));
        }
    }
    bits.extend(rx.finish());
    bits.chunks(params.k as usize).map(<[bool]>::to_vec).collect()
}

fn wrong(sent: &[Vec<bool>], got: &[Vec<bool>]) -> Vec<usize> {
    (0..sent.len()).filter(|&i| got.get(i) != Some(&sent[i])).collect()
}

fn clean(gain: f64) -> Path {
    Path { gain, fir: vec![1.0], sigma: 0.0 }
}

#[test]
fn ideal_codec_carries_every_bit() {
    let p = fixture(None);
    let sent = data(2000, p.k, 1);
    assert_eq!(wrong(&sent, &run(&p, &clean(1.0 / p.gain), Decoding::Viterbi, &sent, 0)), Vec::<usize>::new());
}

#[test]
fn the_precoder_cancels_an_echoing_path() {
    // z2(0) as near 1 as Q0.15 allows; p1(1) = -0.5 undoes h = [1, 0.5].
    let filters = Filters { z1: vec![], p1: vec![-8192], z2: vec![32767], p2: vec![] };
    let p = fixture(Some(filters));
    let sent = data(2000, p.k, 2);
    let path = Path { gain: 1.0 / (p.gain * p.filters.z2[0]), fir: vec![1.0, 0.5], sigma: 0.0 };
    assert_eq!(wrong(&sent, &run(&p, &path, Decoding::Viterbi, &sent, 0)), Vec::<usize>::new());

    let plain = fixture(None);
    let unequalised = Path { gain: 1.0 / plain.gain, ..path };
    assert!(!wrong(&sent, &run(&plain, &unequalised, Decoding::Viterbi, &sent, 0)).is_empty());
}

#[test]
fn an_inverted_line_loses_only_the_first_frames() {
    let p = fixture(None);
    let sent = data(500, p.k, 3);
    let lost = wrong(&sent, &run(&p, &clean(-1.0 / p.gain), Decoding::Viterbi, &sent, 0));
    // Frame 0 by the sign, frame 1's first bits by the descrambler's memory.
    assert!(lost.iter().all(|&f| f < 2), "lost {lost:?}");
}

#[test]
fn trellis_decoding_beats_the_slicer_in_noise() {
    let p = fixture(None);
    let sent = data(2000, p.k, 4);
    let mut found = None;
    for sigma in (4..=80).step_by(4).map(f64::from) {
        let path = Path { gain: 1.0 / p.gain, fir: vec![1.0], sigma };
        let slicer = wrong(&sent, &run(&p, &path, Decoding::Symbolwise, &sent, 9)).len();
        if (100..=600).contains(&slicer) {
            let viterbi = wrong(&sent, &run(&p, &path, Decoding::Viterbi, &sent, 9)).len();
            found = Some((sigma, slicer, viterbi));
            break;
        }
    }
    let (sigma, slicer, viterbi) = found.expect("no noise level gave the slicer 5-30 % frame errors");
    eprintln!("sigma {sigma}: symbolwise {slicer} frames wrong, viterbi {viterbi}, of 2000");
    assert!(2 * viterbi <= slicer, "sigma {sigma}: symbolwise {slicer}, viterbi {viterbi}");
}

#[test]
fn every_rate_runs() {
    // LC levels at the loud end; moduli as large as the classes allow.
    for (drn, lc) in [(1u8, 16u8), (10, 46), (19, 127)] {
        let moduli: [u8; 12] = std::array::from_fn(|i| if i % 4 == 3 { lc } else { (2 * u16::from(lc)).min(255) as u8 });
        let ucodes = (0..lc).map(move |i| 127 - (lc - 1 - i) * if lc > 64 { 1 } else { 2 });
        let p = UpParams::from_cpd(&cpd(drn, moduli, ucodes, None)).unwrap();
        let sent = data(100, p.k, 5);
        assert_eq!(wrong(&sent, &run(&p, &clean(1.0 / p.gain), Decoding::Viterbi, &sent, 0)), Vec::<usize>::new(), "drn {drn}");
    }
}
