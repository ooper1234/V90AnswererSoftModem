//! Scratch (not for commit): how long the PCM-upstream one-shot steps take.
use datapump::v90::ucode::Law;
use datapump::v92::upstream::design::{self, Channel, Limits, Target};
use datapump::v92::upstream::params::UpParams;
use datapump::v92::upstream::receiver::{Decoding, Receiver};
use std::time::Instant;

fn noise(n: usize, seed: u64) -> Vec<f64> {
    let mut s = seed;
    (0..n).map(|_| { s ^= s << 13; s ^= s >> 7; s ^= s << 17; ((s >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0 }).collect()
}

#[test]
#[ignore]
fn time_pcm_one_shots() {
    let signs: Vec<f64> = noise(4096, 7).iter().map(|v| v.signum()).collect();
    let h: Vec<f64> = (0..30).map(|k| 1500.0 * (-(k as f64) / 6.0).exp() * if k % 2 == 0 { 1.0 } else { -0.5 }).collect();
    let rx: Vec<f64> = (0..4096).map(|n| (0..h.len()).filter(|&k| k <= n).map(|k| h[k] * signs[n - k]).sum::<f64>()).collect();
    for (len, taps) in [(4096, 80), (2040, 80)] {
        let t = Instant::now();
        let c = design::estimate(&rx[..len], &signs[..len], taps);
        println!("TIMING estimate {len}x{taps}: {:.1} ms (noise {:.3})", t.elapsed().as_secs_f64() * 1e3, c.noise);
    }
    let c = design::estimate(&rx[..2040], &signs[..2040], 80);
    let t = Instant::now();
    for ratio in [1e-3, 1e-4, 1e-5, 1e-6] { let _ = design::equalise_with(&c.h, 64, 32, ratio); }
    println!("TIMING equalise_with x4 (64,32): {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
    let mut worst = 0.0f64;
    for ratio in [1e-3, 1e-4, 1e-5, 1e-6] {
        let mut job = design::EqualiseJob::new(&c.h, 64, 32, ratio);
        loop {
            let t = Instant::now();
            let done = job.step(1).is_some();
            worst = worst.max(t.elapsed().as_secs_f64() * 1e3);
            if done { break; }
        }
    }
    println!("TIMING EqualiseJob worst single step: {:.2} ms", worst);
    let target = Target { law: Law::Mu, power: 1.0, margin_db: 0.0, limits: Limits { sections: 0, ltot: 192, lmax: 128 }, echo: false };
    let d = design::design(&Channel { h: vec![2000.0], noise: 100.0 }, &target).unwrap();
    let params = UpParams::from_cpd(&d.cpd).unwrap();
    let samples: Vec<f64> = noise(Receiver::WARM, 3).iter().map(|v| v * 3000.0).collect();
    let t = Instant::now();
    let mut r = Receiver::new(params.clone(), Law::Mu, Decoding::Viterbi).unwrap().corrected();
    r.warm(&samples);
    let w = t.elapsed();
    let t2 = Instant::now();
    let mut r2 = Receiver::new(params.clone(), Law::Mu, Decoding::Viterbi).unwrap().corrected().with_pair_offset(2);
    assert!(r2.warm_from(&r));
    println!("TIMING warm_from: {:.3} ms (warm {:.1} ms)", t2.elapsed().as_secs_f64() * 1e3, w.as_secs_f64() * 1e3);
    println!("TIMING warm one candidate: {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
    let t = Instant::now();
    for _ in 0..20 { let _ = Receiver::new(params.clone(), Law::Mu, Decoding::Viterbi).unwrap().corrected(); }
    println!("TIMING 20 receiver news: {:.2} ms", t.elapsed().as_secs_f64() * 1e3);
    let mut r = Receiver::new(params.clone(), Law::Mu, Decoding::Viterbi).unwrap().corrected();
    r.warm(&samples);
    let t = Instant::now();
    for &v in &samples { let _ = r.push_linear(v); }
    println!("TIMING push_linear per sample: {:.1} us", t.elapsed().as_secs_f64() * 1e6 / samples.len() as f64);
}
