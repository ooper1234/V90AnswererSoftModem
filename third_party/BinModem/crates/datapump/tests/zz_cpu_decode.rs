//! Scratch (not for commit): decode CPu bits from the server log and see why
//! Mapping::from_cp refuses them.
use datapump::v90::encoder::Mapping;
use datapump::v90::sequences::Cp;
use datapump::v92::phase4::{CpKind, UpInfo};

fn bits(hex: &str, n: usize) -> Vec<bool> {
    // The log prints 4 bits a digit, the last digit holding what is left.
    let digits: Vec<u32> = hex.chars().map(|c| c.to_digit(16).unwrap()).collect();
    let mut v = Vec::new();
    for (i, d) in digits.iter().enumerate() {
        let w = if i + 1 == digits.len() && n % 4 != 0 { n % 4 } else { 4 };
        v.extend((0..w).rev().map(|k| d >> k & 1 == 1));
    }
    assert_eq!(v.len(), n);
    v
}

#[test]
#[ignore]
fn decode_cpus() {
    for (name, hex) in [
        ("46666 ok", "ffff978140004000401000000000000000000800400842092492aaabffe000000000096a0"),
        ("45333 single DEAF", "ffff938140004000401000000000000000000200080208212492aaa9ffc0000000000bbb2"),
        ("44000 DEAF", "ffff9581400040004010000001100000000000200010100888855553ff80000000000000200010100888855553ff000000000027ff0"),
        ("41333 ok", "ffff968140004000401000000110000000000020001010084222952956c000000000000020001010084222952956800000000004ae0"),
        ("40000 ok", "ffff9281400040004010000001100000000000080002008108424922aaa00000000000000000000000000000000000000000002cb50"),
        ("37333 single ok?", "ffff908140004000401000000000000000000001000020020404888955400000000054400"),
        ("37333 two DEAF", "ffff9081400040004010000001111000000000010000200204048889554000000000000001000020020404888955000000000042480"),
    ] {
        let b = bits(hex, if hex.len() > 80 { 426 } else { 290 });
        let Some(UpInfo::Cp(c)) = UpInfo::from_bits(&b) else { println!("CPU {name}: does not parse"); continue };
        println!("CPU {name}: kind {:?} drn {} red {:?} la {} shaping {:?} ack {} a_law {} trn1d {:?} codec {:?}", c.kind, c.drn, c.redundancy, c.lookahead, c.shaping, c.acknowledge, c.a_law, c.trn1d_ratio, c.codec);
        let cp = Cp { data_mode: c.kind == CpKind::Data, drn: c.drn, silence: false, redundancy: c.redundancy, acknowledge: c.acknowledge, a_law: c.a_law,
            upstream_rates: 0, lookahead: c.lookahead, trn1d_ratio: c.trn1d_ratio, shaping: c.shaping, intervals: c.intervals, constellations: c.constellations.clone(), codec: c.codec.clone() };
        let pts: Vec<usize> = (0..6).map(|i| cp.points(i).len()).collect();
        println!("CPU   intervals {:?}; masks {}: {:?}", cp.intervals, cp.constellations.len(), cp.constellations.iter().map(|m| m.count_ones()).collect::<Vec<_>>());
        for (n, m) in cp.constellations.iter().enumerate() { println!("CPU     mask {n}: {:032x}", m); }
        println!("CPU   points per interval {:?}, frame_bits {}, data_bits(S) {}", pts, cp.frame_bits(), c.redundancy.data_bits());
        match Mapping::from_cp(&cp) { Some(m) => println!("CPU   mapping ok: K {} moduli {:?}", m.frame_bits() - c.redundancy.data_bits(), m.moduli()), None => println!("CPU   mapping REFUSED") }
    }
}
