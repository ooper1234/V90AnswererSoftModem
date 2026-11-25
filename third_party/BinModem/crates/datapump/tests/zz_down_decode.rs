//! Scratch (not for commit): read what the server sent (Asterisk's
//! answerer track) back through the analogue modem's decoder with the
//! mapping the modem asked for, and count intact LAPM frames per second.
use datapump::v32::{Mode, Scrambler};
use datapump::v90::encoder::{Decoder, Mapping};
use datapump::v90::sequences::Cp;
use datapump::v90::ucode::Law;
use datapump::v92::phase4::{CpKind, UpInfo};

fn bits(hex: &str, n: usize) -> Vec<bool> {
    let digits: Vec<u32> = hex.chars().map(|c| c.to_digit(16).unwrap()).collect();
    let mut v = Vec::new();
    for (i, d) in digits.iter().enumerate() {
        let w = if i + 1 == digits.len() && n % 4 != 0 { n % 4 } else { 4 };
        v.extend((0..w).rev().map(|k| d >> k & 1 == 1));
    }
    v
}

fn mapping(hex: &str) -> Mapping {
    let b = bits(hex, if hex.len() > 80 { 426 } else { 290 });
    let Some(UpInfo::Cp(c)) = UpInfo::from_bits(&b) else { panic!("CPu") };
    let cp = Cp { data_mode: c.kind == CpKind::Data, drn: c.drn, silence: false, redundancy: c.redundancy, acknowledge: c.acknowledge, a_law: c.a_law,
        upstream_rates: 0, lookahead: c.lookahead, trn1d_ratio: c.trn1d_ratio, shaping: c.shaping, intervals: c.intervals, constellations: c.constellations.clone(), codec: c.codec.clone() };
    Mapping::from_cp(&cp).unwrap()
}

/// Per second: (flags, frames with a good FCS-16 or -32, frames without).
fn hdlc(bits: &[bool], per_sec: usize) -> Vec<(usize, usize, usize)> {
    let mut out = vec![(0, 0, 0); bits.len() / per_sec + 1];
    let (mut ones, mut cur, mut octet, mut nb) = (0u32, Vec::<u8>::new(), 0u8, 0u32);
    for (i, &b) in bits.iter().enumerate() {
        let s = i / per_sec;
        if b { ones += 1; if ones > 6 { cur.clear(); nb = 0; } continue_bit(&mut octet, &mut nb, &mut cur, true, ones <= 5); continue; }
        if ones == 6 {
            out[s].0 += 1;
            if cur.len() >= 4 {
                if fcs16(&cur) || fcs32(&cur) { out[s].1 += 1 } else { out[s].2 += 1 }
            }
            cur.clear(); nb = 0; octet = 0;
        } else if ones == 5 {
            // stuffed zero: drop
        } else {
            continue_bit(&mut octet, &mut nb, &mut cur, false, true);
        }
        ones = 0;
    }
    out
}

fn continue_bit(octet: &mut u8, nb: &mut u32, cur: &mut Vec<u8>, b: bool, keep: bool) {
    if !keep { return; }
    // A run of ones may still turn out to be a flag; bits pass now and a
    // flag clears the frame, which is near enough for counting.
    *octet = (*octet >> 1) | if b { 0x80 } else { 0 };
    *nb += 1;
    if *nb == 8 { cur.push(*octet); *nb = 0; }
}

fn fcs16(d: &[u8]) -> bool {
    let mut c: u16 = 0xffff;
    for &x in d { c ^= u16::from(x); for _ in 0..8 { c = if c & 1 != 0 { c >> 1 ^ 0x8408 } else { c >> 1 }; } }
    c == 0xf0b8
}

fn fcs32(d: &[u8]) -> bool {
    let mut c: u32 = 0xffff_ffff;
    for &x in d { c ^= u32::from(x); for _ in 0..8 { c = if c & 1 != 0 { c >> 1 ^ 0xedb8_8320 } else { c >> 1 }; } }
    c == 0xdebb_20e3
}

#[test]
#[ignore]
fn decode_downstream() {
    let path = std::env::var("REC").unwrap();
    let raw = std::fs::read(path).unwrap();
    let s: Vec<i32> = raw.chunks_exact(2).map(|b| i32::from(i16::from_le_bytes([b[0], b[1]]))).collect();
    let t0: f64 = std::env::var("T0").unwrap().parse().unwrap();
    let t1: f64 = std::env::var("T1").unwrap().parse().unwrap();
    let m = mapping(&std::env::var("CPU").unwrap());
    let d = m.frame_bits();
    let per_sec = d * 8000 / 6;
    for phase in 0..6 {
        let mut dec = Decoder::new(m.clone());
        let mut sc = Scrambler::new(Mode::Call);
        let from = (t0 * 8000.0) as usize + phase;
        let to = (t1 * 8000.0) as usize;
        let mut out = Vec::new();
        let mut i = from;
        while i + 6 <= to {
            let f: [i32; 6] = std::array::from_fn(|k| s[i + k]);
            for b in dec.from_amplitudes(Law::Mu, f) { out.push(sc.descramble(b)); }
            i += 6;
        }
        let per = hdlc(&out, per_sec);
        let flags: usize = per.iter().map(|p| p.0).sum();
        let good: usize = per.iter().map(|p| p.1).sum();
        println!("DEC phase {phase}: flags {flags} good {good}");
        if good > 0 {
            for (k, p) in per.iter().enumerate() { println!("DEC   {:5.1}s flags {:5} good {:4} bad {:4}", t0 + k as f64, p.0, p.1, p.2); }
        }
    }
}
