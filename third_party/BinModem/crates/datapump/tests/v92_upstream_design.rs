//! V.92 PCM upstream with designed parameters: TRN1u through a telephone
//! band path, the digital modem's estimate and design, then data through the
//! same path with what it designed.

use datapump::v90::ucode::{self, Law};
use datapump::v92::upstream::design::{analogue, design, estimate, trn1u_signs, Limits, Target};
use datapump::v92::upstream::params::UpParams;
use datapump::v92::upstream::receiver::{Decoding, Receiver};
use datapump::v92::upstream::transmitter::Transmitter;

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

/// The analogue modem's line to the codec, and the codec.
struct Path {
    fir: Vec<f64>,
    history: Vec<f64>,
    scale: f64,
    sigma: f64,
    law: Law,
    rng: Rng,
}

impl Path {
    /// `gain` is the level error on top of 2800 codec units per transmit unit:
    /// about -15 dBm0 at the codec, a usual upstream level after the loop.
    fn new(law: Law, gain: f64, sigma: f64, seed: u64) -> Self {
        let fir = telephone_band();
        Self { history: vec![0.0; fir.len()], fir, scale: 2800.0 * gain, sigma, law, rng: Rng(seed) }
    }

    /// One transmit sample in, one codec octet out.
    fn send(&mut self, t: f64) -> u8 {
        self.history.rotate_right(1);
        self.history[0] = t;
        let y: f64 = self.fir.iter().zip(&self.history).map(|(h, x)| h * x).sum::<f64>() * self.scale
            + self.sigma * self.rng.gaussian();
        let (u, negative) = ucode::nearest(self.law, y.round() as i32);
        ucode::octet(self.law, u, negative)
    }

    fn linear(&self, octet: u8) -> f64 {
        let (u, negative) = ucode::from_octet(self.law, octet);
        let v = f64::from(ucode::linear(self.law, u));
        if negative { -v } else { v }
    }
}

struct Outcome {
    took: std::time::Duration,
    drn: u8,
    wrong: Vec<usize>,
    power: f64,
}

fn run(law: Law, gain: f64, sigma: f64, seed: u64) -> Outcome {
    let mut path = Path::new(law, gain, sigma, seed);

    // TRN1u, and the digital modem's view of it.
    let signs = trn1u_signs(4800);
    let octets: Vec<u8> = signs.iter().map(|&s| path.send(s)).collect();
    let heard: Vec<f64> = octets.iter().map(|&o| path.linear(o)).collect();
    let channel = analogue(&estimate(&heard, &signs, 48), &heard, &signs, law);
    let target = Target { law, power: 1.0, margin_db: 0.0, limits: Limits { sections: 0, ltot: 192, lmax: 128 }, echo: false };
    let started = std::time::Instant::now();
    let d = design(&channel, &target).expect("no design");
    let took = started.elapsed();
    let params = UpParams::from_cpd(&d.cpd).expect("the design is not a CPd milestone 2 accepts");

    // Data.
    let mut tx = Transmitter::new(params.clone());
    let mut rx = Receiver::new(params.clone(), law, Decoding::Viterbi).unwrap();
    let mut rng = Rng(seed ^ 0x5555);
    // 1000 frames checked, and 20 more behind them so the codec's delay and
    // the Viterbi decoder's depth end on data, as a call does, not silence.
    let frames: Vec<Vec<bool>> = (0..1020).map(|_| (0..params.k).map(|_| rng.next() & 1 == 1).collect()).collect();
    let mut bits = Vec::new();
    let (mut square, mut count) = (0.0, 0usize);
    let mut skip = d.delay;
    let mut take = |octet: u8, bits: &mut Vec<bool>, rx: &mut Receiver| {
        if skip > 0 {
            skip -= 1;
        } else {
            bits.extend(rx.push_octet(octet));
        }
    };
    for f in &frames {
        for s in tx.frame(f).unwrap().samples {
            square += s * s;
            count += 1;
            let o = path.send(s);
            take(o, &mut bits, &mut rx);
        }
    }
    for _ in 0..d.delay {
        let o = path.send(0.0);
        take(o, &mut bits, &mut rx);
    }
    bits.extend(rx.finish());
    let got: Vec<&[bool]> = bits.chunks(params.k as usize).collect();
    let wrong = (0..1000).filter(|&i| got.get(i).is_none_or(|g| *g != frames[i].as_slice())).collect();
    Outcome { took, drn: d.cpd.drn, wrong, power: square / count as f64 }
}

fn assert_clean(o: &Outcome, what: &str) {
    let late: Vec<usize> = o.wrong.iter().copied().filter(|&f| f >= 2).collect();
    assert!(late.is_empty(), "{what}: drn {}, {} frames wrong after frame 2: {:?}", o.drn, late.len(), &late[..late.len().min(10)]);
}

#[test]
fn designed_parameters_carry_data_over_a_telephone_band() {
    let o = run(Law::Mu, 1.0, 8.0, 1);
    eprintln!("telephone band, mu-law, sigma 8: drn {}", o.drn);
    assert_clean(&o, "mu-law sigma 8");
}

#[test]
fn a_level_error_is_designed_around() {
    for gain in [0.5, 2.0] {
        let o = run(Law::Mu, gain, 8.0, 2);
        eprintln!("gain {gain}: drn {}", o.drn);
        assert_clean(&o, &format!("gain {gain}"));
        // A level error is designed around, not paid for in rate.
        assert!(o.drn >= 8, "gain {gain}: drn {}", o.drn);
    }
}

#[test]
fn a_law_works_too() {
    let o = run(Law::A, 1.0, 8.0, 3);
    eprintln!("A-law: drn {}", o.drn);
    assert_clean(&o, "A-law");
}

#[test]
fn the_transmitter_keeps_to_its_power() {
    let o = run(Law::Mu, 1.0, 8.0, 4);
    assert!(o.power <= 1.5, "mean square {} against 1.0", o.power);
}

#[test]
fn noise_costs_rate_not_errors() {
    let quiet = run(Law::Mu, 1.0, 8.0, 5);
    let loud = run(Law::Mu, 1.0, 20.0, 5);
    eprintln!("sigma 8: drn {}, sigma 20: drn {}", quiet.drn, loud.drn);
    assert!(loud.drn < quiet.drn);
    assert_clean(&loud, "sigma 20");
}

#[test]
fn a_design_is_quick_enough_for_the_start_up() {
    // The design runs between TRN1u and CPd; the start-up cannot wait long.
    if cfg!(debug_assertions) {
        return;
    }
    for (law, sigma) in [(Law::Mu, 8.0), (Law::A, 20.0)] {
        let o = run(law, 1.0, sigma, 6);
        eprintln!("{law:?} sigma {sigma}: design took {:?}", o.took);
        assert!(o.took < std::time::Duration::from_millis(250), "{:?}", o.took);
    }
}
