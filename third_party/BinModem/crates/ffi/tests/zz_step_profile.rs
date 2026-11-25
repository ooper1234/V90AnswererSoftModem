//! Scratch (not for commit): replay a server capture's line input (channel 0)
//! through bm_create_v90/bm_step and time every step, to find what stalls.
use std::os::raw::{c_int, c_void};

unsafe extern "C" fn get_bit(_: *mut c_void) -> c_int { 1 }
unsafe extern "C" fn put_bit(_: *mut c_void, _: c_int) {}

#[test]
#[ignore = "needs a capture"]
fn profile_steps() {
    let Some(path) = std::env::var_os("SRV_CAPTURE") else { return };
    let until: f64 = std::env::var("SRV_UNTIL").ok().and_then(|v| v.parse().ok()).unwrap_or(1e9);
    let b = std::fs::read(path).unwrap();
    let p = b.windows(4).position(|w| w == b"data").unwrap() + 8;
    let ch = u16::from_le_bytes([b[22], b[23]]) as usize;
    let rx: Vec<i16> = b[p..].chunks_exact(2 * ch).map(|f| i16::from_le_bytes([f[0], f[1]])).collect();
    let a = binmodemffi::bm_create_v90(1, Some(get_bit), std::ptr::null_mut(), Some(put_bit), std::ptr::null_mut());
    let mut slow: Vec<(f64, f64)> = Vec::new();
    let mut total = 0.0f64;
    let n = rx.len().min((until * 8000.0) as usize);
    for (i, &x) in rx.iter().take(n).enumerate() {
        let t = std::time::Instant::now();
        binmodemffi::bm_step(a, x as c_int);
        let dt = t.elapsed().as_secs_f64() * 1000.0;
        total += dt;
        if dt >= 5.0 { slow.push((i as f64 / 8000.0, dt)); }
    }
    println!("PROFILE {} samples ({:.1} s of line) in {:.0} ms of CPU; steps >= 5 ms: {}", n, n as f64 / 8000.0, total, slow.len());
    for (t, dt) in &slow { println!("PROFILE slow {:8.3} s  {:6.1} ms", t, dt); }
}
