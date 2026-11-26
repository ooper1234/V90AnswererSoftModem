//! Offline look at a recorded call: where TRN1u's first signs line up in the
//! analogue modem's side of a recording. Ignored unless REPLAY_SLN names a
//! signed 16-bit 8 kHz file.

use datapump::v92::upstream::design::trn1u_signs;

#[test]
#[ignore]
fn where_trn1u_s_first_signs_line_up() {
    let Ok(path) = std::env::var("REPLAY_SLN") else { return };
    let from: f64 = std::env::var("REPLAY_FROM").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
    let to: f64 = std::env::var("REPLAY_TO").ok().and_then(|v| v.parse().ok()).unwrap_or(1e9);
    let bytes = std::fs::read(path).expect("the recording");
    let y: Vec<f64> = bytes.chunks_exact(2).map(|b| f64::from(i16::from_le_bytes([b[0], b[1]]))).collect();
    const FIND: usize = 512;
    let signs = trn1u_signs(FIND);
    let (a, b) = ((from * 8000.0) as usize, ((to * 8000.0) as usize).min(y.len().saturating_sub(FIND)));
    let mut rows: Vec<(f64, usize, f64)> = Vec::new();
    for at in a..b {
        let (mut c, mut e) = (0.0, 0.0);
        for (i, s) in signs.iter().enumerate() {
            c += y[at + i] * s;
            e += y[at + i] * y[at + i];
        }
        rows.push((c.abs() / ((e * FIND as f64).sqrt() + 1e-9), at, c));
    }
    let mut top = rows.clone();
    top.sort_by(|x, y| y.0.total_cmp(&x.0));
    for (s, at, c) in top.iter().take(12) {
        eprintln!("strength {s:.3} at sample {at} ({:.4} s), correlation {c:+.0}", *at as f64 / 8000.0);
    }
    let over = rows.iter().filter(|r| r.0 > 0.3).count();
    eprintln!("{over} windows over 0.3; first: {:?}", rows.iter().find(|r| r.0 > 0.3).map(|r| (r.1, (r.0 * 1000.0).round() / 1000.0)));
}
