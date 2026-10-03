//! Exercise the daemon's 20 ms service cadence with async DTE bytes and LAPM.
use std::collections::VecDeque;
use std::ffi::{c_int, c_void};
use binmodemffi::*;
use datapump::{framing::AsyncBits, v8 as v8line};
use datapump::v90::{network::Network, startup::{Analogue, Status}, ucode::Law};
use ec::{Compression, Params, Role, Stack};
use v8::{CallFunction, Modulation, Modulations, Pcm, PcmRole};

#[derive(Default)]
struct Dte {
    tx: VecDeque<bool>,
    rx: Vec<bool>,
    armed: bool,
}
extern "C" fn get_bit(user: *mut c_void) -> c_int {
    let d = unsafe { &mut *(user as *mut Dte) };
    if d.armed { d.tx.pop_front().unwrap_or(true) as c_int } else { 1 }
}
extern "C" fn put_bit(user: *mut c_void, bit: c_int) {
    let d = unsafe { &mut *(user as *mut Dte) };
    if d.armed { d.rx.push(bit != 0); }
}
enum Far { V8(v8line::Modem), Data(Analogue) }

pub fn run(peer_compression: bool, enable_compression: bool) {
    run_mode(peer_compression, enable_compression, false);
}

pub fn run_fallback() {
    run_mode(true, true, true);
}

fn run_mode(peer_compression: bool, enable_compression: bool, fallback: bool) {
    // The default V.90 path must negotiate error control without a bench hook.
    unsafe {
        std::env::remove_var("V90_ERROR_CONTROL");
        if enable_compression { std::env::remove_var("V90_COMPRESSION"); }
        else { std::env::set_var("V90_COMPRESSION", "0"); }
    }
    let down: Vec<u8> = (0..4096).map(|i| if i < 2048 { b'A' } else { (i * 73 + 19) as u8 }).collect();
    let up: Vec<u8> = (0..4096).map(|i| if i < 2048 { b'B' } else { (i * 151 + 43) as u8 }).collect();
    let mut dte = Dte::default();
    let framing = AsyncBits::new(8);
    for &byte in &down { dte.tx.extend(framing.encode(byte)); }
    let ptr = &mut dte as *mut Dte as *mut c_void;
    let end = bm_create_v90(1, Some(get_bit), ptr, Some(put_bit), ptr);
    assert!(!end.is_null());
    let mut net = Network::new(Law::Mu, 16_000.0).with_delay(0.015, 16_000.0)
        .with_noise(1e-5);
    let mut noise_state = 3490u32;
    let mut far = Far::V8(v8line::Modem::new(v8line::Role::Calling,
        CallFunction::Data, Modulations::of(&[Modulation::V34Duplex]), 16_000.0)
        .offering_pcm(Pcm::ANALOGUE).offering_lapm());
    let mut ec: Option<Stack> = None;
    let mut uplink = Vec::new();
    let mut got_down = Vec::new();
    let mut got_up = Vec::new();
    let mut decode = AsyncBits::new(8);
    let mut sent_up = false;
    let mut far_ready = false;
    let mut far_lapm = false;
    for tick in 0..(60 * 8000) {
        // Poll then service then step, matching sm_call.c. Service throughout
        // negotiation, since V.42 cannot complete without its line bits.
        if tick % 160 == 0 {
            if !dte.armed && bm_status(end) == BM_CONNECTED && far_ready {
                bm_flush_rx(end);
                dte.armed = true;
            }
            bm_service(end);
            for bit in dte.rx.drain(..) {
                if let Some(byte) = decode.feed(bit) { got_up.push(byte); }
            }
        }
        let input = net.up(&uplink);
        uplink.clear();
        let output = bm_step(end, (input * 32768.0).round().clamp(-32768.0, 32767.0) as c_int);
        assert_ne!(bm_status(end), BM_FAILED);
        for mut input in net.down(output as f64 / 32768.0) {
            // Disrupt only initial V.90 training, then allow clean fallback
            // data. Persistent noise would test a different line condition.
            if fallback && (8 * 8000..18 * 8000).contains(&tick) {
                noise_state = noise_state.wrapping_mul(1664525).wrapping_add(1013904223);
                input += ((noise_state >> 16) as f64 / 65535.0 - 0.5) * 0.067;
            }
            match &mut far {
                Far::V8(m) => {
                    uplink.push(0.3 * m.step(input));
                    if matches!(m.status(), v8line::Status::Agreed(_)) {
                        assert_eq!(m.pcm_role(), Some(PcmRole::Analogue));
                        far_lapm = m.lapm();
                        far = Far::Data(Analogue::new(16_000.0));
                    }
                }
                Far::Data(m) => {
                    uplink.push(m.step(input));
                    if let Status::Connected { transmit, .. } = m.status() {
                        if ec.is_none() {
                            let mut stack = Stack::new(Role::Originator, Params {
                                t401_ms: ec::lapm::t401_for(transmit), ..Params::default()
                            });
                            if far_lapm { stack = stack.declared_lapm(); }
                            if peer_compression { stack.offer_compression(Compression::Both); }
                            else { stack.without_v42bis(); }
                            stack.without_v44();
                            ec = Some(stack);
                        }
                        let stack = ec.as_mut().unwrap();
                        for bit in m.take_bits() { stack.feed_bit(bit); }
                        // Two analogue samples for every 8 kHz network tick.
                        if tick % 8 == 0 && uplink.len() == 1 { stack.tick(1); }
                        far_ready = stack.is_connected();
                        if dte.armed && far_ready && !sent_up {
                            stack.send(&up);
                            sent_up = true;
                        }
                        got_down.extend(stack.take_received());
                        if m.accepts_bits() {
                            while m.pending_bits() < 2048 { m.send_bits(&[stack.next_bit()]); }
                        }
                    } else {
                        m.take_bits();
                    }
                }
            }
        }
        if got_down.len() >= down.len() && got_up.len() >= up.len() { break; }
    }
    let connected = dte.armed;
    let lapm = bm_error_control(end);
    let compression = bm_compression(end);
    let rate = bm_rate_tx(end);
    let upstream_rate = bm_rate_rx(end);
    let phase = unsafe { std::ffi::CStr::from_ptr(bm_phase(end)) }.to_string_lossy().into_owned();
    bm_destroy(end);
    assert!(far_lapm, "the digital V.90 modem did not advertise its enabled LAPM");
    assert!(connected, "V.90/LAPM never opened the DTE gate");
    assert_ne!(lapm, 0);
    assert_eq!(compression, i32::from(peer_compression && enable_compression), "negotiated V.42bis");
    if fallback {
        assert!(rate <= 33_600 && upstream_rate <= 33_600);
        assert!(matches!(&far, Far::Data(m) if !m.is_v90()), "caller did not select V.34");
        assert!(phase.starts_with("V.34 data / V.42 / V.42bis"), "wrong fallback phase: {phase}");
    } else if std::env::var("V90_UP_RATE").as_deref() == Ok("28800") {
        assert!(rate >= 48_000);
        assert_eq!(upstream_rate, 28_800, "configured V.90 upstream cap");
    } else {
        assert!(rate >= 48_000);
        assert_eq!(upstream_rate, 33_600, "full-rate V.90 upstream");
    }
    assert_eq!(got_down, down, "downstream bytes");
    assert_eq!(got_up, up, "upstream bytes");
}
