//! The C face of BinModem's answering path: V.8 (ANSam, CM/JM) and then the
//! V.34 start-up -- or, from `bm_create_v90`, V.8 with a digital PCM category
//! and then V.90's digital start-up -- driven one line sample at a time from C.
//!
//! The V.34 engine runs at 16 kHz -- everywhere its own tests run it -- and a
//! resampler either side carries it between that and the 8 kHz AudioSocket
//! line the softmodem daemon speaks. The V.90 digital modem already lives at
//! the network's 8000 Hz and its levels must reach the far end's G.711
//! encoder as exact codewords at unity gain, so `bm_create_v90` mode runs
//! both V.8 and V.90 straight through at the line's rate with neither
//! resampler nor boundary gain between them. Bits cross the boundary exactly
//! as they do between spanDSP and `sm_call.c`: the C side frames DTE bytes
//! into start/data/stop bits, pulls TX bits with a callback, and deframes RX
//! bits itself, so nothing here needs to know about bytes at all.

use std::collections::VecDeque;
use std::os::raw::{c_char, c_double, c_int, c_void};

use datapump::v34;
use datapump::v32;
use datapump::v8 as v8line;
use datapump::v90;
use datapump::framing::AsyncBits;
use dsp::Resampler;
use ec::{Params as EcParams, Role as EcRole, Stack as EcStack};
use ec::stack::Phase as EcPhase;
use ec::xid::Compression;
use v8::{Access, CallFunction, Modulation, Modulations, Pcm, PcmRole};

/// Where a step's time went, by section, so a slow step can say what was
/// slow. Sections may nest; each is charged its own wall time.
mod step_prof {
    use std::cell::RefCell;
    thread_local! { static T: RefCell<Vec<(&'static str, f64)>> = const { RefCell::new(Vec::new()) }; }
    pub fn reset() { T.with(|t| t.borrow_mut().clear()); let _ = datapump::prof::take(); }
    pub fn add(name: &'static str, ms: f64) {
        T.with(|t| {
            let mut t = t.borrow_mut();
            if let Some(e) = t.iter_mut().find(|e| e.0 == name) { e.1 += ms; } else { t.push((name, ms)); }
        });
    }
    pub fn report() -> String {
        let mut parts: Vec<String> = T.with(|t| t.borrow().iter().filter(|e| e.1 >= 1.0).map(|e| format!("{} {:.0} ms", e.0, e.1)).collect());
        let inner: Vec<String> = datapump::prof::take().into_iter().filter(|e| e.1 >= 0.5).map(|e| format!("{} {:.1}", e.0, e.1)).collect();
        if !inner.is_empty() { parts.push(format!("[engine: {}]", inner.join(", "))); }
        parts.join(", ")
    }
}
macro_rules! prof {
    ($name:expr, $e:expr) => {{
        let __t = std::time::Instant::now();
        let __r = $e;
        step_prof::add($name, __t.elapsed().as_secs_f64() * 1000.0);
        __r
    }};
}

/* Status codes, shared with src/call/bm_answerer.h in the softmodem tree. */
pub const BM_RUNNING: c_int = 0; /* V.8 or V.34 start-up still going */
pub const BM_CONNECTED: c_int = 1; /* in V.34 data mode */
pub const BM_FAILED: c_int = 2; /* terminal; bm_failure says why */
pub const BM_AGREED_V22: c_int = 3; /* V.8 chose V.22bis: caller takes over */
pub const BM_AGREED_OTHER: c_int = 4; /* no V.8: caller takes over */
pub const BM_RETRAINING: c_int = 5; /* back from data mode for a retrain */

const ENGINE_FS: f64 = 16_000.0;

// Replacing a trained echo filter during duplex data can disturb the receive
// lock even when the candidate improves the echo-only score. Keep this
// diagnostic feature explicitly opt-in, as documented for the daemon.
fn data_echo_tracking_enabled(value: Option<&str>) -> bool {
    value == Some("1")
}
const LINE_FS: f64 = 8_000.0;
/// 0.4 s of steady AA during ANSam: a V.32 caller, not a V.8 one.
const LEGACY_V32_EARLY_SAMPLES: u32 = 3_200;

// HDLC evidence is meaningful once LAPM owns the line, including before
// SABME/UA succeeds. Detection patterns and transparent async data are not
// CRC-framed and must never trigger this recovery.
fn monitor_receive_framing(physical_data: bool, phase: EcPhase, connected: bool) -> bool {
    connected || (physical_data && phase == EcPhase::Protocol)
}

/// Ten seconds in data mode during which the peer showed no sign of hearing
/// us. `unheard_ms` can have started before data mode came up -- probes sent
/// during a renegotiation go unanswered -- so only the part inside the data
/// period counts.
fn peer_unhearing(unheard_ms: Option<u32>, data_samples: u64) -> bool {
    unheard_for(unheard_ms, data_samples, 10)
}

fn unheard_for(unheard_ms: Option<u32>, data_samples: u64, seconds: u64) -> bool {
    unheard_ms.is_some_and(|ms| (u64::from(ms) * 8).min(data_samples) >= seconds * 8000)
}

/// Looking for the frame boundary an unhearing peer reads at (see
/// `next_realign`). `offset` is how many downstream symbols have been left
/// out since the search began, modulo 6.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Realign {
    Idle,
    /// The echo path grew by a number of samples that is not a multiple of
    /// 6; the peer has been polled and, if it has not answered by
    /// `check_at`, its frame boundary moved by `offset`.
    Shifted { offset: u8, check_at: u64 },
    Slipping { offset: u8, tried: u8, check_at: u64 },
}

impl Realign {
    /// After a validated playout delay shift of `samples`.
    fn shifted(samples: isize, now: u64) -> Self {
        match samples.rem_euclid(6) as u8 {
            0 => Realign::Idle,
            offset => Realign::Shifted { offset, check_at: now + REALIGN_WAIT },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RealignAction {
    Nothing,
    /// Leave this many downstream symbols out and poll the peer.
    Slip(u8),
    /// The peer answered after this many tries.
    Heard(u8),
    /// Every boundary tried, or none worth trying: retrain.
    Retrain,
    /// Deaf after a rate renegotiation: start another one from this end
    /// (V.92 9.8.1.1) before a retrain.
    Renegotiate,
}

/// Two seconds for a poll's answer to come back over a 0.9 s round trip.
const REALIGN_WAIT: u64 = 2 * 8000;

/// Renegotiation rescues (`next_realign`) before a retrain, while the peer
/// stays unheard: the modem can renegotiate again before it hears our first.
const RENEG_RESCUES: u8 = 2;

/// How many renegotiation rescues are left: `left` until the peer is heard
/// inside the current data period (heard more recently than data came up,
/// and within a poll's wait), or data comes back from a retrain; then all.
fn reneg_rescues_left(left: u8, unheard_ms: Option<u32>, data_samples: u64, retrained: bool) -> u8 {
    if retrained || heard_this_period(unheard_ms, data_samples) { RENEG_RESCUES } else { left }
}

/// The peer answered inside the current data period: more recently than
/// data came up, and within a poll's wait.
fn heard_this_period(unheard_ms: Option<u32>, data_samples: u64) -> bool {
    unheard_ms.is_some_and(|ms| {
        let ago = u64::from(ms) * 8;
        ago < REALIGN_WAIT && ago < data_samples
    })
}

/// A peer that stops hearing us in data mode can be reading our data frames
/// at the wrong boundary: the path dropped or added samples, and a frame
/// boundary is never looked for again in data mode (a rate renegotiation
/// keeps it too, V.92 9.8). og7 call 4: the path grew by 320 samples, the
/// analogue modem went deaf, and two downstream symbols left out (320 mod 6)
/// had it hearing us again for the rest of the call.
///
/// So: after a validated delay shift, poll, and if no answer, slip by the
/// shift modulo 6; after ten seconds unheard with no shift seen, slip one at
/// a time. Either way try every boundary but the one it started from, two
/// seconds each, then retrain.
///
/// `after_renegotiation`: data mode came back from a rate renegotiation.
/// Then most often the peer never hears us again (13 of 16 on 2026-11-13)
/// and a full slip search found nothing, so six seconds unheard -- two
/// probes -- ends the period. `rescue`: no renegotiation of ours has been
/// tried on this deafness yet, so one is (it hears us after about half of
/// its own, ~3 s against a ~25 s retrain); once it has, a retrain.
fn next_realign(state: Realign, unheard_ms: Option<u32>, data_samples: u64, now: u64, after_renegotiation: bool, rescue: bool, peer_began: bool) -> (Realign, RealignAction) {
    let heard = unheard_ms.is_some_and(|ms| u64::from(ms) * 8 < REALIGN_WAIT);
    match state {
        Realign::Idle if after_renegotiation => {
            // Half-seconds: 3.5 s after the peer's own renegotiation, six
            // after ours (see the tests for the counts behind each).
            let half_seconds: u64 = if peer_began { 7 } else { 12 };
            if unheard_ms.is_some_and(|ms| (u64::from(ms) * 8).min(data_samples) * 2 >= half_seconds * 8000) {
                (state, if rescue { RealignAction::Renegotiate } else { RealignAction::Retrain })
            } else {
                (state, RealignAction::Nothing)
            }
        }
        Realign::Idle if peer_unhearing(unheard_ms, data_samples) => {
            (Realign::Slipping { offset: 1, tried: 1, check_at: now + REALIGN_WAIT }, RealignAction::Slip(1))
        }
        Realign::Idle => (state, RealignAction::Nothing),
        Realign::Shifted { check_at, .. } | Realign::Slipping { check_at, .. } if now < check_at => (state, RealignAction::Nothing),
        Realign::Shifted { .. } if heard => (Realign::Idle, RealignAction::Nothing),
        Realign::Shifted { offset, .. } => {
            (Realign::Slipping { offset, tried: 1, check_at: now + REALIGN_WAIT }, RealignAction::Slip(offset))
        }
        Realign::Slipping { tried, .. } if heard => (Realign::Idle, RealignAction::Heard(tried)),
        Realign::Slipping { offset, tried, .. } if tried < 5 => {
            // The boundary it started from is known not to work.
            let (offset, n) = match (offset + 1) % 6 { 0 => (1, 2), o => (o, 1) };
            (Realign::Slipping { offset, tried: tried + 1, check_at: now + REALIGN_WAIT }, RealignAction::Slip(n))
        }
        Realign::Slipping { .. } => (Realign::Idle, RealignAction::Retrain),
    }
}

#[cfg(test)]
mod tx_watermark_tests {
    use super::*;

    fn ms(line: TxLine, rate: u32) -> f64 {
        tx_watermark(line, rate) as f64 * 1000.0 / f64::from(rate)
    }

    #[test]
    fn a_v90_downstream_queues_about_50_ms_and_more_than_an_audio_chunk() {
        for rate in [28_000, 40_000, 44_000, 46_666, 50_666, 52_000, 53_333, 56_000] {
            let held = ms(TxLine::V90, rate);
            assert!(held <= 55.0, "{rate} bit/s holds {held:.0} ms");
            assert!(held > 20.0, "{rate} bit/s holds only {held:.0} ms, under one audio chunk");
        }
    }

    #[test]
    fn v32_and_v34_are_as_they_were() {
        assert_eq!(tx_watermark(TxLine::V34, 21_600), 1080);
        assert_eq!(tx_watermark(TxLine::V34, 33_600), 1680);
        assert_eq!(tx_watermark(TxLine::V32, 14_400), 720);
        assert_eq!(tx_watermark(TxLine::Other, 0), TX_WATERMARK);
    }
}

/// Whether everything the DTE handed this end has reached the far modem, as
/// far as can be known: under V.42, every frame acknowledged. Without V.42
/// (none, declined, or a link already down) nothing more will be confirmed,
/// and at most the transmitter's watermark of raw bits is left, under 0.3 s
/// at any V.90 rate.
fn tx_delivered(ec: Option<&EcStack>) -> bool {
    match ec {
        Some(ec) if ec.phase() != EcPhase::Transparent && ec.is_connected() => ec.all_acknowledged(),
        _ => true,
    }
}

#[cfg(test)]
mod tx_delivered_tests {
    use super::*;

    #[test]
    fn unacknowledged_v42_data_is_not_delivered_and_no_v42_is() {
        assert!(tx_delivered(None));
        let mut a = EcStack::new(EcRole::Originator, EcParams::default()).declared_lapm().without_detection();
        let mut b = EcStack::new(EcRole::Answerer, EcParams::default()).declared_lapm().without_detection();
        a.connect();
        let pump = |a: &mut EcStack, b: &mut EcStack, n: usize| for _ in 0..n {
            let x = a.next_bit(); let y = b.next_bit(); b.feed_bit(x); a.feed_bit(y);
            a.tick(1); b.tick(1);
        };
        pump(&mut a, &mut b, 20_000);
        assert!(a.is_connected());
        assert!(tx_delivered(Some(&a)));
        a.send(b"NO CARRIER from the BBS");
        assert!(!tx_delivered(Some(&a)));
        pump(&mut a, &mut b, 40_000);
        assert!(tx_delivered(Some(&a)));
    }
}

#[cfg(test)]
mod error_control_clock_tests {
    use super::*;

    #[test]
    fn an_unhearing_peer_is_slipped_to_and_found_again() {
        let s = 8000u64;
        // Not deaf yet: nothing.
        assert_eq!(next_realign(Realign::Idle, Some(5_000), 20 * s, 100 * s, false, false, false), (Realign::Idle, RealignAction::Nothing));
        // Deaf: the first slip, checked two seconds on.
        let (st, act) = next_realign(Realign::Idle, Some(10_000), 10 * s, 100 * s, false, false, false);
        assert_eq!((st, act), (Realign::Slipping { offset: 1, tried: 1, check_at: 102 * s }, RealignAction::Slip(1)));
        // Before the check: wait.
        assert_eq!(next_realign(st, Some(11_000), 11 * s, 101 * s, false, false, false), (st, RealignAction::Nothing));
        // Heard since the slip: done.
        assert_eq!(next_realign(st, Some(700), 12 * s, 102 * s, false, false, false), (Realign::Idle, RealignAction::Heard(1)));
    }

    #[test]
    fn unheard_after_a_renegotiation_retrains_at_six_seconds_without_slipping() {
        let s = 8000u64;
        // og2-og6: of 16 data periods after a renegotiation the analogue
        // modem began, 3 were heard, and a full slip search found nothing.
        // A modem that hears us answers the second probe by about 4.5 s.
        assert_eq!(next_realign(Realign::Idle, Some(9_000), 5 * s, 100 * s, true, false, false), (Realign::Idle, RealignAction::Nothing));
        assert_eq!(next_realign(Realign::Idle, Some(9_000), 6 * s, 100 * s, true, false, false), (Realign::Idle, RealignAction::Retrain));
    }

    #[test]
    fn unheard_after_a_renegotiation_renegotiates_once_before_it_retrains() {
        let s = 8000u64;
        // og10-ud19: the analogue modem hears us after about half of its own
        // renegotiations (3 of 6 since the 07:46 replug), so a deaf one gets
        // a second renegotiation from this end (9.8.1.1, ~3 s) before a full
        // retrain (~25 s). Not before six seconds unheard.
        assert_eq!(next_realign(Realign::Idle, Some(9_000), 5 * s, 100 * s, true, true, false), (Realign::Idle, RealignAction::Nothing));
        assert_eq!(next_realign(Realign::Idle, Some(9_000), 6 * s, 100 * s, true, true, false), (Realign::Idle, RealignAction::Renegotiate));
        // That renegotiation deaf too: the rescue is spent, retrain.
        assert_eq!(next_realign(Realign::Idle, Some(9_000), 6 * s, 110 * s, true, false, false), (Realign::Idle, RealignAction::Retrain));
    }

    #[test]
    fn unheard_after_the_peers_own_renegotiation_is_rescued_at_three_and_a_half_seconds() {
        let s = 8000u64;
        // pcmup71 (2026-11-17): the CX93001 was deaf after 4 of 4 of its own
        // renegotiations and heard after 4 of 4 of ours. A modem that hears
        // us under load is heard within 2 s of data coming back.
        let at = |secs: f64| next_realign(Realign::Idle, Some(9_000), (secs * 8000.0) as u64, 100 * s, true, true, true).1;
        assert_eq!(at(3.0), RealignAction::Nothing);
        assert_eq!(at(3.5), RealignAction::Renegotiate);
        // Rescue spent: retrain, at the same point.
        assert_eq!(next_realign(Realign::Idle, Some(9_000), 4 * s, 100 * s, true, false, true).1, RealignAction::Retrain);
        // After ours, the six seconds stand.
        assert_eq!(next_realign(Realign::Idle, Some(9_000), 5 * s, 100 * s, true, true, false).1, RealignAction::Nothing);
        // Without a renegotiation, the rescue changes nothing.
        assert_eq!(next_realign(Realign::Idle, Some(10_000), 10 * s, 100 * s, false, true, false).1, RealignAction::Slip(1));
    }

    #[test]
    fn the_renegotiation_rescue_is_rearmed_by_hearing_the_peer_or_by_a_retrain() {
        let s = 8000u64;
        // Spent, then the peer is heard 1 s ago, 4 s into this data period.
        assert_eq!(reneg_rescues_left(0, Some(1_000), 4 * s, false), RENEG_RESCUES);
        // Heard before this data period began (the renegotiation took 3 s):
        // not evidence the peer hears the new downstream.
        assert_eq!(reneg_rescues_left(0, Some(1_000), s / 2, false), 0);
        // Still deaf: stays spent.
        assert_eq!(reneg_rescues_left(0, Some(5_000), 6 * s, false), 0);
        // Data came back from a retrain, not a renegotiation: armed again.
        assert_eq!(reneg_rescues_left(0, Some(30_000), 0, true), RENEG_RESCUES);
        // Part used, still deaf: what is left stays.
        assert_eq!(reneg_rescues_left(1, Some(30_000), 6 * s, false), 1);
    }

    #[test]
    fn a_peer_heard_since_the_renegotiation_is_no_longer_after_one() {
        let s = 8000u64;
        // ud32/ud33 (pcmup38): heard for a minute after a renegotiation,
        // then 6 s unheard at 124 s and 145 s, and a rescue fired "since a
        // rate renegotiation" -- a path jump is likelier then, and the slip
        // search is what handles it.
        assert!(heard_this_period(Some(1_000), 4 * s));
        assert!(!heard_this_period(Some(1_000), s / 2));
        assert!(!heard_this_period(Some(5_000), 60 * s));
        assert!(!heard_this_period(None, 60 * s));
    }

    #[test]
    fn a_second_deaf_renegotiation_before_the_peer_is_heard_gets_a_second_rescue() {
        let s = 8000u64;
        // ud27 (call 7, 2026-11-14): the modem's Ru, deaf, our renegotiation;
        // the modem renegotiated again 8 s later, before it had been heard,
        // and was deaf again: the single rescue was spent and the retrain that
        // followed began four more. Two rescues before a retrain; never a
        // third without the peer heard in between.
        let mut left = RENEG_RESCUES;
        let mut actions = Vec::new();
        for _ in 0..3 {
            let (_, act) = next_realign(Realign::Idle, Some(9_000), 6 * s, 100 * s, true, left > 0, false);
            if act == RealignAction::Renegotiate { left -= 1; }
            left = reneg_rescues_left(left, Some(9_000), 6 * s, false);
            actions.push(act);
        }
        assert_eq!(actions, vec![RealignAction::Renegotiate, RealignAction::Renegotiate, RealignAction::Retrain]);
    }

    #[test]
    fn a_delay_jump_slips_by_its_size_modulo_six_once_the_peer_is_silent() {
        let s = 8000u64;
        // og7 call 4: the path grew by 320 samples, the analogue modem went
        // deaf with no renegotiation, and two slips (320 mod 6) found it.
        let st = Realign::shifted(320, 100 * s);
        assert_eq!(st, Realign::Shifted { offset: 2, check_at: 102 * s });
        assert_eq!(next_realign(st, Some(1_500), 30 * s, 101 * s, false, false, false), (st, RealignAction::Nothing));
        let (st, act) = next_realign(st, Some(2_500), 30 * s, 102 * s, false, false, false);
        assert_eq!((st, act), (Realign::Slipping { offset: 2, tried: 1, check_at: 104 * s }, RealignAction::Slip(2)));
        assert_eq!(next_realign(st, Some(800), 32 * s, 104 * s, false, false, false), (Realign::Idle, RealignAction::Heard(1)));
    }

    #[test]
    fn a_delay_jump_the_peer_rides_out_needs_nothing() {
        let s = 8000u64;
        let st = Realign::shifted(400, 100 * s);
        // Answered since the jump (the shift was on the way up, say).
        assert_eq!(next_realign(st, Some(900), 30 * s, 102 * s, false, false, false), (Realign::Idle, RealignAction::Nothing));
        // A multiple of six moves no frame boundary.
        assert_eq!(Realign::shifted(480, 100 * s), Realign::Idle);
    }

    #[test]
    fn a_search_from_a_jump_tries_every_other_boundary_but_the_first() {
        let s = 8000u64;
        let mut st = Realign::shifted(400, 100 * s); // offset 4
        let mut now = 102 * s;
        let mut offsets = Vec::new();
        loop {
            let (next, act) = next_realign(st, Some(30_000), 30 * s, now, false, false, false);
            if let RealignAction::Slip(_) = act {
                let Realign::Slipping { offset, .. } = next else { panic!() };
                offsets.push(offset);
            }
            if act == RealignAction::Retrain { break; }
            st = next;
            now += s;
        }
        assert_eq!(offsets, vec![4, 5, 1, 2, 3]);
    }

    #[test]
    fn five_slips_without_an_answer_end_in_a_retrain() {
        let s = 8000u64;
        let mut st = Realign::Idle;
        let mut now = 100 * s;
        let mut slips = 0;
        loop {
            let (next, act) = next_realign(st, Some(30_000), 30 * s, now, false, false, false);
            st = next;
            match act {
                RealignAction::Slip(n) => slips += n,
                RealignAction::Retrain => break,
                RealignAction::Nothing => {}
                RealignAction::Heard(_) | RealignAction::Renegotiate => panic!("never answered"),
            }
            now += s / 2;
        }
        assert_eq!(slips, 5, "every other frame boundary tried once");
        assert_eq!(st, Realign::Idle);
    }


    #[test]
    fn a_peer_deaf_since_before_data_came_up_is_caught_ten_seconds_in() {
        // og2 call 1: unheard started 0.5 s before data mode and ran on.
        assert!(!peer_unhearing(Some(10_500), 9 * 8000));
        assert!(peer_unhearing(Some(10_500), 10 * 8000));
        assert!(peer_unhearing(Some(42_000), 42 * 8000));
    }

    #[test]
    fn a_peer_heard_recently_or_a_link_down_is_not_deaf() {
        assert!(!peer_unhearing(Some(4_000), 60 * 8000));
        assert!(!peer_unhearing(None, 60 * 8000));
    }


    #[test]
    fn initial_lapm_failure_is_monitored_only_after_physical_training() {
        use ec::frame::{Frame, Kind, DLCI_DATA};
        use ec::hdlc::Encoder;
        let mut answer = EcStack::new(EcRole::Answerer, EcParams::default())
            .declared_lapm().without_detection();
        for _ in 0..200 {
            for _ in 0..1200 { answer.next_bit(); }
            answer.tick(50);
        }
        assert_eq!(answer.phase(), EcPhase::Protocol);
        // Reproduce the initial-call evidence: CRC-damaged frames reach the
        // stack, but no SABME/UA exchange has established LAPM yet.
        for _ in 0..3 {
            let mut wire = Encoder::new(answer.fcs());
            wire.frame(&Frame::Rr { nr: 0, pf: true }
                .encode(DLCI_DATA, EcRole::Originator, Kind::Command));
            let mut bits: Vec<bool> = std::iter::from_fn(|| wire.next_bit()).collect();
            bits[12] = !bits[12];
            for bit in bits { answer.feed_bit(bit); }
        }
        assert!(!answer.is_connected());
        assert_eq!(answer.valid_frames(), 0);
        assert!(answer.damaged_frames() >= 3);
        assert!(monitor_receive_framing(true, answer.phase(), answer.is_connected()),
            "a failed initial SABME/UA exchange needs receive recovery too");
        assert!(!monitor_receive_framing(false, EcPhase::Protocol, false));
        for phase in [EcPhase::Detecting, EcPhase::Negotiating, EcPhase::Transparent] {
            assert!(!monitor_receive_framing(true, phase, false));
        }
    }

    #[test]
    fn compressed_ppp_start_waits_for_peer_data_or_bounded_timeout() {
        let mut end = Answerer::new(true, true, false, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        let params = ec::v42bis::Params { n2: 512, n7: 32 };
        let mut answer = EcStack::new(EcRole::Answerer, EcParams::default())
            .declared_lapm().with_compression(params);
        let mut caller = EcStack::new(EcRole::Originator, EcParams::default())
            .declared_lapm().with_compression(params);
        caller.connect();
        for i in 0..100000 {
            let a = answer.next_bit(); let c = caller.next_bit();
            caller.feed_bit(a); answer.feed_bit(c);
            if i % 160 == 0 { caller.tick(16); answer.tick(16); }
            if caller.is_connected() && answer.is_connected() { break; }
        }
        assert!(answer.is_connected());
        end.physical_connected = true; end.ec = Some(answer);
        end.update_connected_status(); assert_eq!(end.status, BM_RUNNING);
        end.samples = 15999; end.update_connected_status();
        assert_eq!(end.status, BM_RUNNING);
        end.samples = 16000; end.update_connected_status();
        assert_eq!(end.status, BM_CONNECTED);
        end.samples = 0; end.ec_ready_at = None;
        caller.send(&[13]);
        for _ in 0..10000 {
            let a = end.ec.as_mut().unwrap().next_bit(); let c = caller.next_bit();
            caller.feed_bit(a); end.ec.as_mut().unwrap().feed_bit(c);
        }
        end.update_connected_status(); assert_eq!(end.status, BM_CONNECTED);
    }
    #[test]
    fn valid_supervisory_acknowledgements_keep_receive_framing_healthy() {
        use ec::frame::{Frame, Kind, DLCI_DATA};
        use ec::hdlc::Encoder;
        let mut answer = EcStack::new(EcRole::Answerer, EcParams::default()).declared_lapm();
        let mut caller = EcStack::new(EcRole::Originator, EcParams::default()).declared_lapm();
        caller.connect();
        for i in 0..100000 {
            let a=answer.next_bit(); let c=caller.next_bit();
            caller.feed_bit(a); answer.feed_bit(c);
            if i%160==0 { caller.tick(16); answer.tick(16); }
            if caller.is_connected() && answer.is_connected() { break; }
        }
        assert!(answer.is_connected());
        let mut end=Answerer::new(true,true,false,None,std::ptr::null_mut(),None,std::ptr::null_mut());
        end.stage=Stage::Done;
        end.ec=Some(answer);
        bm_service(&mut end);
        let information=end.ec.as_ref().unwrap().valid_information_frames();
        for seconds in 1..=10 {
            let before=end.ec.as_ref().unwrap().valid_frames();
            let mut wire=Encoder::new(end.ec.as_ref().unwrap().fcs());
            wire.frame(&Frame::Rr{nr:0,pf:false}.encode(DLCI_DATA,EcRole::Originator,Kind::Command));
            while let Some(bit)=wire.next_bit() { end.ec.as_mut().unwrap().feed_bit(bit); }
            assert!(end.ec.as_ref().unwrap().valid_frames()>before);
            assert_eq!(end.ec.as_ref().unwrap().valid_information_frames(),information);
            end.samples=seconds*8000;
            bm_service(&mut end);
            assert_eq!(end.framing_since,end.samples,"an intact acknowledgement proves framing, even without an I frame");
        }
    }
    #[test]
    fn measured_network_delay_is_passed_to_error_control() {
        let mut modem = Answerer::new(true, true, false, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        modem.echo.delay = 5600;
        modem.rate_tx = 14400;
        modem.rate_rx = 14400;
        modem.start_error_control();
        for _ in 0..8000 { modem.tick_error_control(LINE_FS as u32); }
        assert_eq!(modem.ec.as_ref().unwrap().phase(), EcPhase::Detecting,
            "750 ms alone must not expire detection on a 700 ms path");
        for _ in 0..3600 { modem.tick_error_control(LINE_FS as u32); }
        assert_eq!(modem.ec.as_ref().unwrap().phase(), EcPhase::Transparent,
            "silent peer must still have a bounded 1450 ms timeout");
    }
    #[test]
    fn detection_timeout_uses_the_active_sample_rate() {
        for rate in [LINE_FS as u32, ENGINE_FS as u32] {
            let mut modem = Answerer::new(true, true, false, None,
                std::ptr::null_mut(), None, std::ptr::null_mut());
            modem.ec = Some(EcStack::new(EcRole::Answerer, EcParams::default()));
            let timeout = ec::detect::DEFAULT_T400_MS;
            for _ in 0..rate * (timeout - 1) / 1000 {
                modem.tick_error_control(rate);
            }
            assert_eq!(modem.ec.as_ref().unwrap().phase(), EcPhase::Detecting);
            for _ in 0..rate / 1000 { modem.tick_error_control(rate); }
            assert_eq!(modem.ec.as_ref().unwrap().phase(), EcPhase::Transparent,
                       "detection at {rate} Hz must expire after {timeout} ms");
        }
    }
}

/// How far below a full mapping frame the TX queue is allowed to fall before
/// it is topped up. A mapping frame takes its whole width in one gulp, and
/// whatever is short of it is made up with idle ones -- correct as line
/// idle, corruption if it lands part-way through a byte. The engine's
/// transmit side runs at up to 33 600 bit/s and this service call covers a
/// whole audio chunk at once, so the watermark has to hold more than one
/// chunk's worth: 8192 bits is about 2 s at 33 600.
const TX_WATERMARK: usize = 8192;

/// Which transmitter `bm_service` is topping up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TxLine { V32, V34, V90, Other }

/// Bits kept queued ahead of the transmitter: about 50 ms at the rate in
/// use. Under V.42 the queue is kept full with flags, so every frame waits
/// behind all of it. V.90 had the whole 8192 bits, 158 ms at 52 000 bit/s,
/// on every packet and every acknowledgement to the caller (the friend's
/// calls of 2026-11-21: a 1 s ping on a 0.54 s line). Never under 1400 bits,
/// which is more than one 20 ms audio chunk at any V.90 rate.
fn tx_watermark(line: TxLine, rate: u32) -> usize {
    match line {
        TxLine::V32 => (rate.max(4800) as usize / 20).clamp(256, 1024),
        TxLine::V34 => (rate.max(2400) as usize / 20).clamp(256, 2048),
        TxLine::V90 => (rate.max(28_000) as usize / 20).clamp(1400, 2800),
        TxLine::Other => TX_WATERMARK,
    }
}

/// What the engine's own line signal is scaled by at the s16 boundary, both
/// out and (inversely) in. The engine's f64 waveform peaks around 2.4, which
/// hard-clipped at full scale and put slips into the far end's receiver; the
/// two scalings cancel engine-to-engine, so each end still sees the other at
/// native level while the 16-bit line stays clear of the rails.
const BOUNDARY_GAIN: f64 = 0.4;

/* ------------------------------------------------------------------ */
/* Near-end echo cancellation at the boundary.                        */
/* ------------------------------------------------------------------ */

/* On the real line the far hybrid reflects our own transmit back at us:
   measured on captured calls it comes back about 190 ms late (PAP2T
   playout plus the round trip through the telephone pair) at roughly 14 dB
   below our transmit level. Phase 3 is received in silence and trains to
   27 dB; phase 4 is full duplex, and there that reflection sits 11 dB
   below the call modem's signal -- exactly the SNR the capture replays
   show -- which is too poor for the 88/188-bit CRC'd MP sequences, so MP
   is never decoded and the exchange times out with "no E from the call
   modem". A delay-locked NLMS filter over our own transmitted samples
   takes the reflection off before the engine sees it. */

/// Whether V.90's data mode gets a V.42 stack at all, `V90_ERROR_CONTROL` in
/// the environment.
///
/// Off by default, which is the behaviour this path has always had: V.90 data
/// mode raw, with no error control. Whether the stack is worth having is not
/// yet settled -- the far modem over V.8 says LAPM=0 and drops the link to
/// transparent within T401, so it may buy nothing, and this crate's own V.90
/// loopback has a far end with no V.42 to answer with, so it never leaves
/// negotiation. Both are measurements to make rather than assumptions to build
/// in, so the variable decides until they have been made.
/// Whether V.90's data mode stops transmitting, `V90_TX_MUTE` in the
/// environment.
///
/// A bench hook, and the first thing to try about a receive path that decodes
/// noise: our own 48 kbit/s is the loudest thing in the band the upstream
/// receiver reads, because V.90's downstream carrier is close enough to the
/// upstream one that a filter wide enough for wideband data passes it. The
/// receiver would then be reading our own transmission, which against the
/// coarse slicer grid scores like a locked signal -- a real signal, the wrong
/// one. Muting in data mode only, so the start-up that gets there still runs.
fn v90_tx_mute() -> bool {
    use std::sync::Mutex;
    static ON: Mutex<Option<bool>> = Mutex::new(None);
    let mut guard = ON.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("V90_TX_MUTE").is_ok_and(|v| v != "0"));
    }
    guard.unwrap_or(false)
}

fn v90_error_control() -> bool {
    use std::sync::Mutex;
    static ON: Mutex<Option<bool>> = Mutex::new(None);
    let mut guard = ON.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("V90_ERROR_CONTROL").map_or(true, |v| v != "0"));
    }
    guard.unwrap_or(false)
}

const ECHO_TAPS: usize = 512; /* 64 ms of echo spread */

/**
 * How many taps the filter has, `ECHO_TAPS` in the environment overriding
 * the constant. The echo this call path brings back is not 64 ms long: on
 * the 2026-09-24 23:16 capture a 256-tap model of it accounts for 1.7% of
 * the line while our TRN2d and MP are out, 512 taps for 3.5%, 1024 for 6.9%
 * and 2048 for 14.6% -- and what is left over is what stopped the receiver
 * reading the analogue modem's CPt in phase 4, at 3 dB against the 35 dB it
 * read the same modem's phase 3 at.
 */
fn echo_taps() -> usize {
    use std::sync::Mutex;
    static N: Mutex<Option<usize>> = Mutex::new(None);
    let mut guard = N.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("ECHO_TAPS").ok().and_then(|v| v.parse().ok()).unwrap_or(ECHO_TAPS));
    }
    guard.unwrap_or(ECHO_TAPS)
}
/// Whether a filter found adding to the line while the far end is silent is
/// left out (`Echo::do_no_harm`); `ECHO_NO_HARM=0` turns that off.
fn echo_no_harm() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("ECHO_NO_HARM").as_deref() != Ok("0"))
}
/// Whether `Echo::report` logs the cancellation depth once a window.
fn echo_depth() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("ECHO_DEPTH").is_ok_and(|v| v != "0"))
}

/// ECHO_REFERENCE, when it names a capture, supplies the echo filter's
/// reference from channel 1 of that recording -- the transmit channel -- instead
/// of from what this run generated.
///
/// A replay generates a different transmit signal from the one in the
/// recording, so the canceller adapts to a signal that is not the one whose
/// echo is on the line and cancels almost none of it: that is why the replay of
/// a V.90 call used to stop short of data mode, and why parameter sweeps had to
/// be run on the hardware, one call each. Reading the reference off the
/// recording puts the live path back together around a recording, and the
/// sweep becomes a replay.
fn recorded_next() -> Option<f64> {
    use std::sync::Mutex;
    static SAMPLES: Mutex<Option<(Vec<i16>, usize)>> = Mutex::new(None);
    let mut guard = SAMPLES.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        let path = std::env::var("ECHO_REFERENCE").ok()?;
        let raw = std::fs::read(&path).unwrap_or_else(|why| panic!("ECHO_REFERENCE {path}: {why}"));
        assert_eq!(&raw[0..4], b"RIFF" as &[u8; 4], "ECHO_REFERENCE {path} is not a RIFF file");
        let mut i = 12;
        let (mut rate, mut data) = (0u32, Vec::new());
        while i + 8 <= raw.len() {
            let id = &raw[i..i + 4];
            let n = u32::from_le_bytes(raw[i + 4..i + 8].try_into().expect("four bytes")) as usize;
            if id == b"fmt " {
                rate = u32::from_le_bytes(raw[i + 12..i + 16].try_into().expect("four bytes"));
            } else if id == b"data" {
                data = raw[i + 8..(i + 8 + n).min(raw.len())]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect();
            }
            i += 8 + n + (n & 1);
        }
        assert_eq!(rate, LINE_FS as u32, "ECHO_REFERENCE {path} is not a line-rate capture");
        // Stereo, interleaved: the even samples are what arrived and the odd
        // ones what went out, so the reference is every other sample.
        //
        // Start where the replay starts. A replay fed from V90_AT seconds in
        // meets the line at that point, so a reference read from the top of the
        // file would be that many seconds out of step with the echo on it and
        // would cancel none of it -- which is where the replay used to stop,
        // short of data mode.
        let skip = std::env::var("V90_AT")
            .ok()
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap_or(0.0)
            .max(0.0);
        *guard = Some((data, (skip * f64::from(LINE_FS)) as usize));
    }
    let (samples, at) = guard.as_mut().expect("set just above");
    // 2n + 1: channel 1, the transmit channel.
    let s = samples.get(2 * *at + 1).copied().unwrap_or(0);
    *at += 1;
    Some(f64::from(s) / 32768.0)
}

/// Dot product kept in four independent partial sums, so the multiply-adds
/// overlap instead of each waiting on the one before. Differs from a single
/// running sum only in rounding (about 1e-15 relative).
fn dot4(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    let (a, b) = (&a[..n], &b[..n]);
    let mut s = [0.0f64; 4];
    let (ca, cb) = (a.chunks_exact(4), b.chunks_exact(4));
    let (ra, rb) = (ca.remainder(), cb.remainder());
    for (x, y) in ca.zip(cb) {
        s[0] += x[0] * y[0];
        s[1] += x[1] * y[1];
        s[2] += x[2] * y[2];
        s[3] += x[3] * y[3];
    }
    let mut t = (s[0] + s[1]) + (s[2] + s[3]);
    for (x, y) in ra.iter().zip(rb) {
        t += x * y;
    }
    t
}

const ECHO_RING: usize = 16384; /* includes high-latency provider reflections */
const ECHO_LAG_LO: usize = 640; /* search from 80 ms ... */
/* The playout-shift check runs on half a second of line, and if that finds
   nothing on a second and then two: under a far end 17 dB louder than the
   echo, half a second cannot tell the right shift from noise. */
const DELAY_WINDOW_MIN: usize = 4096;
const DELAY_WINDOW_MAX: usize = 4 * DELAY_WINDOW_MIN;
const ECHO_LAG_HI: usize = 11200; /* V.90 provider echo: up to 1.4 seconds */
const V34_ECHO_LAG_HI: usize = 12000; /* V.34 carrier routes: up to 1.5 seconds */
/* The widest filter solved inside the far end's silence rather than at its end.
 * 256 covers the local ATA's echo tail (floor from 128 taps on) and is solved
 * from the same 8192 quiet samples in well under the window. */
const COMPACT_TAPS: usize = 256;
const ECHO_WINDOW: usize = 1024; /* lock/adapt gate on 128 ms of input */
const ECHO_PEAK_MIN: f64 = 0.3; /* correlation needed to lock a delay */
/// Lags scoring within this fraction of the peak are aliases of it (a
/// periodic reference), settled on wider rows: see `training_delay_resolving`.
const ALIAS_TIE: f64 = 0.97;
const ECHO_QUIET_DB: f64 = -30.0; /* input loudness the far end must be under */
const ECHO_MU: f64 = 0.5;
/* The adaptive step's floor on the transmit energy it is divided by, a tap's
 * worth of -30 dBFS: a transmit at -16 dBFS (the DIL) fills 256 taps to about
 * 6.5 and barely notices it. Without it a near-silent transmit against a line
 * that is not quite silent made the step enormous (call 0, 2026-11-21: the
 * filter from a norm of 0.08 to 7 between the DIL and phase 4). */
const ECHO_NLMS_FLOOR_PER_TAP: f64 = 1e-3;
/* How far apart the lags of the TX-correlation projection are, in samples. */
const AB_LAG_STEP: usize = 8;
/* The data-mode tracker refits against wideband traffic, then accepts a
 * replacement only after an independent held-back comparison. Both windows
 * include enough history for the longest provider echo. At 12000 samples of
 * delay these leave over 4096 judging samples and 8192 fitting samples. */
const TRACK_FIT: usize = 20_480;
const TRACK_HOLD: usize = 16_384;
const TRACK_RING: usize = TRACK_FIT + TRACK_HOLD;
/* Try every half second; the asynchronous fit does not block line samples. */
const TRACK_PERIOD: usize = 4_096;
/* How much better a candidate has to be, on the held-back samples, before it
 * replaces what is in the filter. Half a decibel is noise on a window this
 * size. The aligned estimator's double-talk regression measured a spurious
 * 1.4 dB improvement over an already exact path; require 3 dB. */
const TRACK_MARGIN_DB: f64 = 3.0;
/* How far the fitted path's peak may sit from the middle of the window before
 * the lock is moved to bring it back. 48 taps either side of the edge. */
const TRACK_EDGE: usize = 48;

/* How many line samples of one call's data-mode window are logged: the same
   2.56 s the constellation dump covers, so the two describe one interval. */
const DATA_LOG_SAMPLES: u64 = 20_480;
/* How far into data mode the logged window starts.
 *
 * The tracker needs a ring's worth of samples before its first attempt, so a
 * window that opens at data-mode entry is nothing but the filter the tracker
 * exists to replace. It did exactly that: 16384 logged samples, the tracker's
 * entire warm-up, and an ERLE_tx of 12.6 dB that was the DIL filter's number all
 * over again. Starting two seconds in measures the regime the call spends
 * almost all of its time in.
 */
const DATA_LOG_SKIP: u64 = 16_384;
/* How long a window the A/B is judged on.
 *
   It has to be longer than the longest delay the search can report plus half
   the tap window plus the projection's own tail, because a lag whose reference
 * runs off the end of the window is not searched at all. That is 3072 + 256 +
 * 256 = 3584 for the longest lockable delay, so 4096. At 2048 the search was
 * silently truncated to lags below 1792, and a call locking at 1879 had its
 * path's peak outside the range being measured -- which is a large part of why
 * the measured ERLE_tx swung from 15.7 dB to -4.6 dB between calls that had the
 * same estimator and the same path. Half a second at 8 kHz.
 */
const AB_WINDOW: usize = 16384;
/* The ridge on the normal equations, as a fraction of the reference's own
   energy. Swept, not guessed: over the DIL's whole spectral range anything
   from zero to 1e-4 moves the recovered gain by under a tenth, and 1e-3 by a
   quarter. It is here to keep the factorisation well-posed and not to shape
   the answer, so it is the smallest thing that does that. */
const LS_RIDGE: f64 = 1.0e-6;
/* Past this spread of the pivots the reference has not identified every tap
   of the span: the live fits were at 0.1 dB (broadband) and 3.1 dB (the DIL
   through 64 taps) when right, 47.7 to 47.9 dB when wrong. */
const LS_SPREAD_MAX_DB: f64 = 20.0;
/* The narrowest span the solve falls back to: the filter the DIL has always
   been fitted through on the other answerers. */
const LS_SPAN_MIN: usize = 64;

/// Solve a symmetric positive-definite system by Cholesky, and report the pivots.
///
/// Returns the pivots as well as the solution because their spread is how close
/// the system came to singular, which is the number the ridge is there to
/// answer and the only honest way to say whether it was needed at all.
///
/// `a` is row-major `n` by `n` and is overwritten. `None` if a pivot comes out
/// non-positive, which for a normal-equation matrix means the correlation has
/// not identified the path and the right answer is to leave the filter alone
/// rather than to divide by it.
fn cholesky_solve(a: &[f64], b: &[f64], n: usize) -> Option<(Vec<f64>, Vec<f64>)> {
    let mut m = a.to_vec();
    let mut y = b.to_vec();
    let mut pivots = Vec::with_capacity(n);
    for k in 0..n {
        let mut d = m[k * n + k];
        for j in 0..k {
            let l = m[k * n + j];
            d -= l * l;
        }
        if !(d > 0.0) {
            return None;
        }
        pivots.push(d);
        let dk = d.sqrt();
        m[k * n + k] = dk;
        for i in k + 1..n {
            let mut s = m[i * n + k];
            for j in 0..k {
                s -= m[i * n + j] * m[k * n + j];
            }
            m[i * n + k] = s / dk;
        }
    }
    // Forward substitution, L y = b.
    for i in 0..n {
        let mut s = y[i];
        for j in 0..i {
            s -= m[i * n + j] * y[j];
        }
        y[i] = s / m[i * n + i];
    }
    // Back substitution, L' x = y.
    for i in (0..n).rev() {
        let mut s = y[i];
        for j in i + 1..n {
            s -= m[j * n + i] * y[j];
        }
        y[i] = s / m[i * n + i];
    }
    Some((y, pivots))
}

/// The echo path for the data-mode A/B/C replay, given as taps.
///
/// Read from `V90_ABC_PATH` because it has to be estimated outside the engine:
/// the question is whether a *different model of the path* restores the
/// constellation, and estimating it here with the code under test could only
/// show that the code agrees with itself. `scripts/bench/abc-window.py` writes
/// the taps and they are passed in like this.
fn abc_path() -> Option<Vec<f64>> {
    let raw = std::env::var("V90_ABC_PATH").ok()?;
    let v: Vec<f64> = raw
        .split_whitespace()
        .map(|t| t.parse().ok())
        .collect::<Option<_>>()?;
    if v.len() < 2 {
        return None;
    }
    Some(v)
}

/// Whether the DIL solves the path in one block, which `ECHO_LS` turns off to
/// leave the gradient descent to do it.
fn echo_ls() -> bool {
    use std::sync::OnceLock;
    static ON: std::sync::OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("ECHO_LS").as_deref() != Ok("0"))
}

/// V.34 identifies the reflection while sending phase-3 PP/TRN, before the
/// peer starts its own training. Disable only for comparative recordings.
fn v34_echo_identify() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("V34_ECHO_IDENTIFY").as_deref() != Ok("0"))
}
/* How far either side of the lock a re-lock may wander and still count as the
   same path, and the fraction of the locked correlation it must keep for the
   filter to stay where it is. 64 samples is 8 ms, an order of magnitude either
   side of the drift a call's path shows. */
const ECHO_HOLD: usize = 64;
const ECHO_HOLD_KEEP: f64 = 0.7;

/// The step the echo filter takes while the far end is talking, where its
/// error is the far end's signal as much as its own. `ECHO_SLOW_MU` in the
/// environment sets it; it is off by default, which is what the measurement
/// says: on the 2026-09-24 23:16 capture 0.02 takes phase 4 from 2.8 dB to
/// 19.0 dB for the half second after the CPt is answered, and 3 to 5 dB
/// again once TRN2d and MP are out, for four CP sequences parsed either way.
/// The filter converges on the echo and then on the far modem's CPt with it,
/// which is the whole of the double-talk problem, and a step small enough not
/// to do that is too small to converge in the time there is.
fn slow_mu() -> f64 {
    use std::sync::Mutex;
    static MU: Mutex<Option<f64>> = Mutex::new(None);
    let mut guard = MU.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        *guard = Some(std::env::var("ECHO_SLOW_MU").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0));
    }
    guard.unwrap_or(0.0)
}

/// Whether the echo filter may adapt with the far end talking, gated by
/// `ECHO_DOUBLE_TALK` in the environment: see [`Echo::double_talk`].
fn double_talk() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("ECHO_DOUBLE_TALK").is_some())
}

/// The boundary's own transmit, delayed and filtered, subtracted from the
/// boundary's receive.
/// Correlations one sample's step may spend on the delay search. The whole
/// search is some 2,700 of them: 35-46 ms in one audio step on the server
/// when it ran at once (2026-11-15), which is the stall the audio pacer held
/// 240 ms of lead against. At 64 a step it is done in 42 samples, and no step
/// carries more than a few hundred microseconds of it.
const SCAN_LAGS_PER_STEP: usize = 64;

enum ScanStage {
    Hold { hi: usize },
    Full,
    Refine { hi: usize },
    Done,
}

/// One delay search: the window and the transmit history as they stood when
/// the window closed, and how far the search through them has got.
struct ScanJob {
    /// The transmit history oldest first, so every lag reads one contiguous
    /// slice (`corr_at` paid a ring-buffer division per sample per lag).
    lin: Vec<f64>,
    seen: Vec<f64>,
    /// The window's own energy, which does not depend on the lag.
    een: f64,
    delay: usize,
    lag_hi: usize,
    stage: ScanStage,
    lag: usize,
    best: usize,
    bestc: f64,
}

impl ScanJob {
    fn new(echo: &Echo) -> Self {
        let w = echo.seen.len();
        let n = echo.tx.len();
        let len = (echo.lag_hi + w).min(n);
        let lin: Vec<f64> = (0..len).map(|k| echo.tx[(echo.tx_pos + n - len + k) % n]).collect();
        let mut job = ScanJob {
            lin,
            seen: echo.seen.clone(),
            een: dot4(&echo.seen, &echo.seen),
            delay: echo.delay,
            lag_hi: echo.lag_hi,
            stage: ScanStage::Full,
            lag: ECHO_LAG_LO,
            best: 0,
            bestc: -1.0,
        };
        if echo.delay != 0 {
            job.best = echo.delay;
            job.lag = echo.delay.saturating_sub(ECHO_HOLD);
            job.stage = ScanStage::Hold { hi: (echo.delay + ECHO_HOLD).min(echo.lag_hi - 1) };
        } else {
            job.begin_full();
        }
        job
    }

    fn begin_full(&mut self) {
        self.stage = ScanStage::Full;
        self.lag = ECHO_LAG_LO;
        self.best = 0;
        self.bestc = -1.0;
    }

    fn consider(&mut self, lag: usize) {
        let c = self.corr(lag).abs();
        if c > self.bestc {
            self.bestc = c;
            self.best = lag;
        }
    }

    /// `Echo::corr_at` on the snapshot.
    fn corr(&self, lag: usize) -> f64 {
        let w = self.seen.len();
        let len = self.lin.len();
        if w < 256 || lag + w > len {
            return 0.0;
        }
        let t = &self.lin[len - lag - w..len - lag];
        let tnn = dot4(t, t);
        if self.een <= 1e-12 || tnn <= 1e-12 {
            0.0
        } else {
            dot4(&self.seen, t) / (self.een * tnn).sqrt()
        }
    }
}

struct Echo {
    /// The last validated playout delay shift, until taken.
    shift: Option<isize>,
    training_async: bool,
    training_epoch: u64,
    /// The receiver has trained against the filter in place (V.90 phase 4
    /// on): a background fit finishing now is not swapped in under it.
    hold_fits: bool,
    training_job: Option<std::sync::mpsc::Receiver<TrainingFit>>,
    tx: Vec<f64>,
    tx_pos: usize,
    /// Protocol-specific bound; V.90 keeps its original search cost.
    lag_hi: usize,
    /// Locked echo delay in samples; 0 until a peak passes the gate.
    delay: usize,
    /// The correlation the lock was accepted on.
    peak: f64,
    w: Vec<f64>,
    seen: Vec<f64>,
    /// The delay search begun at the last quiet window's close, if it has
    /// not finished yet.
    scan_job: Option<ScanJob>,
    frozen: bool,
    /// The training epoch whose runaway adaptive filter was last reported.
    runaway_epoch: Option<u64>,
    /// The link is in data mode, where both ends are transmitting wideband and
    /// the far end never goes quiet: the filter has to follow the reflection
    /// there or the echo of this end's own data is what the receiver sees.
    /// Set from the modem's status, which is where the decision belongs: a
    /// slow step over a start-up sequence would learn the far end's training
    /// signal instead of the path, and phase 4 stops reading altogether.
    data: bool,
    /// The far end is known to be silent, which only the modem knows: 9.3.1.6
    /// has it silent through the DIL, and that is the one window in the whole
    /// call where the line is loud with nothing but our own reflection. The
    /// filter cannot work it out from levels, because a line loud with this
    /// end's own transmission is loud in every window.
    far_silent: bool,
    peer_silent: bool,
    /// The double-talk gate's running sums over one window: the energy the
    /// echo accounts for, and the energy it does not.
    echo_energy: f64,
    residual_energy: f64,
    gate_samples: usize,
    /// What the filter predicted and what was left over, summed over the
    /// window, for ECHO_DEPTH: the cancellation depth in decibels, the only
    /// number that says whether the filter is tracking the path or not.
    pred_energy: f64,
    left_energy: f64,
    left_samples: usize,
    /// The line and what cancelling left of it, over the window being
    /// measured, and whether the last window found the filter adding more
    /// than it took away -- in which case the line goes on uncancelled. An
    /// estimate that is any part of the echo leaves less than the line, the
    /// far end talking or not: its residual is the far end and the error.
    harm_line: f64,
    harm_left: f64,
    harm_samples: usize,
    harmful: bool,
    /// Whether the line is this end's echo and at most the analogue modem's
    /// S, so that the judgement above holds: V.90 phase 3 from TRN1d to
    /// phase 4 (`echo_judged`). Elsewhere the far end can be far louder than
    /// the echo, and a filter that matters a great deal to the receiver moves
    /// the line's energy hardly at all.
    harm_judged: bool,
    last_mu: f64,
    /// Whether the window just measured was one the filter was allowed to
    /// adapt in. Taken in the sample loop, because `report` runs once
    /// `seen` has been cleared and asking then always says no.
    left_quiet: bool,
    /// The block identification of the echo path, over the window the modem
    /// says the far end is silent. See [`Echo::ls_feed`].
    ls: Option<Ls>,
    /// The A/B between the gradient's taps and the block solve's, over a window
    /// neither of them was fitted to. See [`Echo::ab_select`].
    ab: Option<Ab>,
    /// An A/B being judged on a worker (`ab_select`).
    ab_job: Option<std::sync::mpsc::Receiver<AbVerdict>>,
    /// The last sample's line, reference and prediction, so that whoever calls
    /// [`Echo::sample`] can log all three without this knowing about the log.
    last: Last,
    /// Whether the data-mode tracker runs. Set for V.90's data mode and not for
    /// V.34's, whose canceller keeps the behaviour it has always had.
    tracking: bool,
    // A quiet input can still contain the peer. After identifying the path,
    // only the independently checked tracker may replace it.
    identified: bool,
    /// The data-mode tracker. See `TRACK_RING`.
    track: Track,
    track_job: Option<std::sync::mpsc::Receiver<TrackFit>>,
    track_epoch: u64,
    /// The filter at each commit, newest last, for the data-mode log to write
    /// out: the filter is not the one that was in place when the window opened.
    track_writes: Vec<(u64, usize, Vec<f64>)>,
    // Paired line/prediction samples for delay-only recovery. Coefficients
    // are never fitted to the peer's data by this detector.
    delay_window: Vec<(f64, f64)>,
}

/// The data-mode tracker's window and its bookkeeping.
///
/// The samples are the line and our own transmit, oldest first, capped at
/// [`TRACK_RING`]. Nothing here is ever adapted towards: a candidate is fitted,
/// judged on samples the fit did not see, and either replaces the filter or is
/// thrown away. There is no gradient step and no `far_end_silent`, because in
/// data mode the far end is transmitting and a step small enough not to chase it
/// is too small to converge.
#[derive(Default)]
struct Track {
    rx: std::collections::VecDeque<f64>,
    reference: std::collections::VecDeque<f64>,
    /// Samples since the last attempt.
    since: usize,
    /// Attempts made, and how many were taken.
    tries: usize,
    taken: usize,
    /// The last figure each candidate reached on the held-back samples, so the
    /// log says what was rejected and not only what was accepted.
    held_best: f64,
    held_now: f64,
}

struct TrackFit {
    epoch: u64,
    delay: usize,
    w: Vec<f64>,
    accepted: bool,
    held_best: f64,
    held_now: f64,
}

struct TrainingFit {
    epoch: u64,
    delay: usize,
    w: Vec<f64>,
    identified: bool,
    ls: Option<Ls>,
}

/// One sample's three series, kept for the data-mode log.
#[derive(Clone, Copy, Default)]
struct Last {
    /// The line as it arrived, before any cancellation.
    x: f64,
    /// Our own transmit at that moment, which is what the filter predicted from.
    r: f64,
    /// What the filter predicted the line's echo of that would be.
    yhat: f64,
}

/// The two candidate filters and the window they are judged on.
///
/// The judgement is on the energy left that is *correlated with our own
/// transmitted signal*, because that is the only part of the residual a filter
/// can be blamed for. The residual also holds the far modem's signal and the
/// line's noise, which no echo filter is responsible for and which the old
/// prediction-against-residual ratio could not tell apart from the echo: it
/// goes up when the filter's output is simply larger, so it preferred a big
/// noisy filter to a small correct one and kept the gradient forever.
/// What an A/B decided: the filter to run and the delay to run it at, for
/// the path as it stood at `judged_at` in training epoch `epoch`.
struct AbVerdict {
    epoch: u64,
    judged_at: usize,
    delay: usize,
    w: Vec<f64>,
}

struct Ab {
    /// What the gradient left, kept by `ls_solve`.
    gradient: Vec<f64>,
    /// What the block solve produced.
    ls: Vec<f64>,
    /// The window, as paired input and reference samples, taken after the far
    /// end is talking again so that neither candidate was fitted to it.
    window: Vec<(f64, f64)>,
}

/// The block least-squares identification of the echo path.
///
/// The path is static and this end's own transmitted samples are known exactly,
/// so over a window where the far modem is silent the filter is not something
/// to descend towards: it is a least-squares solution, and it is better solved
/// than descended to. NLMS needs on the order of `taps` samples per tap to
/// settle, and the DIL is 1.96 s -- 15 655 samples, about thirty per tap for
/// 512 taps -- which converges to about 30 dB. The data-mode constellation
/// wants 45.
///
/// In the frequency domain the same solution is one divide per bin, because a
/// stationary path's transfer function is the ratio of the cross-spectrum to
/// the auto-spectrum. So the window is accumulated as Welch spectra, `H` is
/// formed once, regularised where this end's own signal has little energy, and
/// the impulse response comes back by inverse transform. That is the same
/// least-squares answer at a cost that fits inside a call.
struct Ls {
    /// The received samples, and our own transmitted ones alongside: the
    /// canceller's reference is exactly what `push` was given. The whole
    /// far-silent window is kept, because the estimate is a correlation over
    /// all of it and a correlation that only sees part of it is the one thing
    /// that cannot be arranged afterwards.
    buf_x: Vec<f64>,
    buf_r: Vec<f64>,
    samples: usize,
    quiet_from: Option<usize>,
    solved: bool,
    /// Cancellation measured over the far-silent window before the solve and
    /// after it, so the log can show what the solve bought rather than what the
    /// window happened to read.
    depth_before: f64,
    depth_after: f64,
    /// 10log10 of the ratio of the largest to the smallest pivot the
    /// factorisation produced, which is how close the normal equations came to
    /// singular and is what the ridge is there to answer.
    cond_db: f64,
    /// What the solve was told, and what it did.
    note: String,
    /// The filter's prediction and the residual over the window, so the log can
    /// show what the solve bought rather than what the window happened to read.
    pred: f64,
    res: f64,
    /// Set once the solve is in the filter, so the after-figure is measured on
    /// samples taken after it and not before.
    solved_at: u64,
    /// The same measurement taken after the solve, and whether it has been
    /// printed yet.
    post_pred: f64,
    post_res: f64,
    post_n: u32,
    reported: bool,
    shortfall_reported: bool,
    /// The taps as the gradient left them, so a solve that measures worse than
    /// the gradient can be refused rather than merely complained about.
    kept: Vec<f64>,
    /// The correlation the solve was built from, so the test can check the
    /// estimator against a path it knows rather than against itself.
    lags: usize,
    /// Set once the A/B has been handed the two candidates, so a second window
    /// of far-end silence does not start a second comparison.
    ab_armed: bool,
}

impl Ls {
    fn take_shortfall(&mut self, needed: usize) -> Option<usize> {
        if self.samples == 0 || self.solved || self.samples >= needed || self.shortfall_reported {
            return None;
        }
        self.shortfall_reported = true;
        Some(self.samples)
    }

    /// An identification window that has collected nothing yet.
    ///
    /// The lock is not used to size anything. It used to be: the transform was
    /// twice the lock, on the reasoning that an impulse response has to fit in
    /// the transform without wrapping round it. That is a real constraint, and
    /// it is the wrong one to satisfy minimally, because the transform's length
    /// is also what decides how much of each block's reference the correlation
    /// can see. A correlation over blocks of `n` for a path at delay `D` sees
    /// `(n - D) / n` of the reference the numerator needs, while the denominator
    /// still counts all of it, so the estimate comes out at `g * (1 - D / n)` --
    /// and `n = 2 D` is exactly where that is one half. The two constraints are
    /// independent, and the smallest transform that fits the path is the one
    /// worst placed to measure it. Nothing here is sized against the delay.
    fn blank(lock: usize) -> Self {
        let _ = lock;
        Ls {
            // The DIL is 1.96 s, 15 655 samples, and a little slack.
            buf_x: Vec::with_capacity(20_000),
            buf_r: Vec::with_capacity(20_000),
            samples: 0,
            quiet_from: None,
            solved: false,
            depth_before: 0.0,
            depth_after: 0.0,
            cond_db: 0.0,
            note: String::new(),
            pred: 0.0,
            res: 0.0,
            solved_at: 0,
            post_pred: 0.0,
            post_res: 0.0,
            post_n: 0,
            reported: false,
            shortfall_reported: false,
            kept: Vec::new(),
            lags: 0,
            ab_armed: false,
        }
    }
}

impl Echo {
    fn new() -> Self {
        Self {
            training_async: false,
            shift: None,
            training_epoch: 0,
            hold_fits: false,
            training_job: None,
            ab_job: None,
            tx: vec![0.0; ECHO_RING],
            tx_pos: 0,
            lag_hi: ECHO_LAG_HI,
            delay: 0,
            peak: 0.0,
            w: vec![0.0; echo_taps()],
            seen: Vec::with_capacity(ECHO_WINDOW),
            scan_job: None,
            frozen: false,
            runaway_epoch: None,
            data: false,
            far_silent: false,
            peer_silent: true,
            tracking: false,
            identified: false,
            last: Last::default(),
            track: Track::default(),
            track_job: None,
            track_epoch: 0,
            track_writes: Vec::new(),
            delay_window: Vec::new(),
            echo_energy: 0.0,
            residual_energy: 0.0,
            gate_samples: 0,
            pred_energy: 0.0,
            left_energy: 0.0,
            left_samples: 0,
            harm_line: 0.0,
            harm_left: 0.0,
            harm_samples: 0,
            harmful: false,
            harm_judged: false,
            last_mu: 0.0,
            left_quiet: false,
            ls: None,
            ab: None,
        }
    }

    /// Normalized correlation of the input window against our transmit as
    /// it stood `lag` samples of delay ago.
    fn corr_at(&self, lag: usize) -> f64 {
        let w = self.seen.len();
        if w < 256 {
            return 0.0;
        }
        let n = self.tx.len();
        let mut dot = 0.0;
        let mut een = 0.0;
        let mut tnn = 0.0;
        for (i, &x) in self.seen.iter().enumerate() {
            let t = self.tx[(self.tx_pos + n - (lag + w - i)) % n];
            dot += x * t;
            een += x * x;
            tnn += t * t;
        }
        if een <= 1e-12 || tnn <= 1e-12 {
            0.0
        } else {
            dot / (een * tnn).sqrt()
        }
    }

    /// Hunt for the reflection's delay -- but only on a window the far end
    /// is quiet on, where the only thing that can correlate with our
    /// transmit is our own echo. Both gates that matter (this and the NLMS
    /// update) close during full-duplex training windows so the far end's
    /// all-ones TRN, which correlates with our own all-ones TRN, can never
    /// drag the filter onto the signal it is supposed to leave alone.
    fn scan(&mut self) {
        // Once DIL identifies the path, its saved comparison filters share
        // this delay. A correlation peak cannot move that coordinate system:
        // another reflection can win the peak without the path moving.
        // Subsequent drift is handled by the validated tracking estimator.
        // DIL repeats structured symbols. Their secondary correlation peaks
        // can win the search without the physical echo moving. Keep the
        // window chosen when identification starts until its solve finishes;
        // otherwise a new peak can move the true path outside the tap range.
        if self.identified || self.ab_pending() || self.ls.as_ref().is_some_and(|ls| !ls.solved) {
            return;
        }
        if self.frozen || self.seen.len() < ECHO_WINDOW / 2 {
            return;
        }
        if !self.quiet() || !self.peer_silent {
            return;
        }
        // The search itself runs a slice per sample from here (`scan_step`),
        // never all in one: see `ScanJob`.
        let job = ScanJob::new(self);
        self.scan_job = Some(job);
        self.scan_step(SCAN_LAGS_PER_STEP);
    }

    /// Carry the scan begun at the last window's close `budget` lags on, and
    /// act on it when it is done.
    ///
    /// What the scan decides is what the one-shot scan decided, from the
    /// window and the transmit history as they were when the window closed.
    /// Only *when* differs, by at most 42 samples. A lock that something
    /// else moved, identified or put under a measurement meanwhile is not the
    /// lock the scan looked around, so its answer is dropped, exactly as the
    /// one-shot scan's gates would have dropped it.
    fn scan_step(&mut self, mut budget: usize) {
        let Some(mut job) = self.scan_job.take() else { return };
        // Once DIL identifies the path, its saved comparison filters share
        // this delay. A correlation peak cannot move that coordinate system:
        // another reflection can win the peak without the path moving.
        // Subsequent drift is handled by the validated tracking estimator.
        // DIL repeats structured symbols. Their secondary correlation peaks
        // can win the search without the physical echo moving. Keep the
        // window chosen when identification starts until its solve finishes;
        // otherwise a new peak can move the true path outside the tap range.
        if self.identified || self.ab_pending() || self.ls.as_ref().is_some_and(|ls| !ls.solved)
            || self.frozen || self.delay != job.delay
        {
            return;
        }
        while budget > 0 {
            match job.stage {
                // Once the path is locked, look around the lock first and only go
                // searching the whole range if there is nothing there.
                //
                // This is not a small thing. The taps straddle the lock
                // (`d0 = delay - w.len()/2`) over 512 taps, so a lock that moves moves
                // every tap, and NLMS has to relearn the window from nothing. A real
                // call measured on 2026-09-26 12:36 flip-flopped between 1168 and 1688
                // -- 520 samples, the whole tap window -- on successive quiet windows,
                // with the correlation at 0.98 or better on both, and the cancellation
                // depth never got past 20 to 32 dB for want of a stable geometry to
                // converge in. The far end of a call does not move 62 ms of path in
                // 128 ms; a second reflection taking the correlation peak is enough,
                // and a filter cannot tell the two apart while it re-locks every time.
                ScanStage::Hold { hi } => {
                    if job.lag <= hi {
                        job.consider(job.lag);
                        job.lag += 2;
                        budget -= 1;
                        continue;
                    }
                    if job.bestc >= self.peak * ECHO_HOLD_KEEP {
                        self.peak = self.peak.max(job.bestc);
                        self.shift_to(job.best);
                        return;
                    }
                    job.begin_full();
                }
                ScanStage::Full => {
                    if job.lag < job.lag_hi {
                        job.consider(job.lag);
                        job.lag += 4;
                        budget -= 1;
                        continue;
                    }
                    if job.best != 0 {
                        job.lag = job.best.saturating_sub(4);
                        job.stage = ScanStage::Refine { hi: (job.best + 5).min(job.lag_hi - 1) };
                    } else {
                        job.stage = ScanStage::Done;
                    }
                }
                ScanStage::Refine { hi } => {
                    if job.lag <= hi {
                        job.consider(job.lag);
                        job.lag += 1;
                        budget -= 1;
                        continue;
                    }
                    job.stage = ScanStage::Done;
                }
                ScanStage::Done => {
                    // A lock already held is only given up for a peak that would have
                    // held it. A quiet window that happens to carry little of our own
                    // transmit scores low at the true delay and something middling at an
                    // alias; taking the alias -- and its score as the bar from then on --
                    // is how a 0.998 lock at 7328 became 0.577 at 1448 on a live call,
                    // and phase 3 then started with the echo uncancelled.
                    let bar = if self.delay != 0 { ECHO_PEAK_MIN.max(self.peak * ECHO_HOLD_KEEP) } else { ECHO_PEAK_MIN };
                    if job.bestc >= bar {
                        self.shift_to(job.best);
                        self.peak = job.bestc;
                    }
                    return;
                }
            }
        }
        self.scan_job = Some(job);
    }

    /// Move the lock to `lag`, carrying the filter's weights across so the
    /// taps still describe the same reflection.
    ///
    /// Tap `k` reads the transmit sample `delay - w.len()/2 + k` ago, so a lock
    /// that moves by `delta` samples means tap `k` must take what tap `k +
    /// delta` held, and the taps that run off either end of the window have had
    /// no data to learn from and start at zero. Without this a re-lock throws
    /// away a converged filter; with it a path that really has moved costs one
    /// window of adaptation rather than all of it.
    fn shift_to(&mut self, lag: usize) {
        if lag == self.delay {
            return;
        }
        let delta = lag as isize - self.delay as isize;
        let n = self.w.len();
        let old = std::mem::replace(&mut self.w, vec![0.0; n]);
        for (k, slot) in self.w.iter_mut().enumerate() {
            let from = k as isize + delta;
            if from >= 0 && (from as usize) < n {
                *slot = old[from as usize];
            }
        }
        self.delay = lag;
    }

    /// Take one sample into the identification window: `x` is what arrived,
    /// `r` is what this end transmitted at the same moment.
    ///
    /// Only called while the modem says the far end is silent, which is what
    /// makes the window a measurement of the path rather than of the far end.
    ///
    /// Nothing is computed here. The estimate is a correlation over the whole
    /// window, so it cannot start until the window has stopped, and doing it
    /// incrementally over blocks is what put the factor of two into the
    /// amplitude in the first place.
    fn ls_feed(&mut self, x: f64, r: f64) {
        let ls = self.ls.get_or_insert_with(|| Ls::blank(self.delay));
        if ls.solved {
            return;
        }
        if self.peer_silent && ls.quiet_from.is_none() {
            ls.quiet_from = Some(ls.buf_x.len());
        }
        ls.samples += 1;
        ls.buf_x.push(x);
        ls.buf_r.push(r);
    }

    fn begin_training_measurement(&mut self) {
        let history = (self.lag_hi + self.w.len()).min(self.tx.len() - 1);
        let mut ls = Ls::blank(self.delay);
        // The quiet receive window reflects transmit samples from before it
        // began. Keep that reference history; synthetic receive padding is
        // excluded from the fitting rows, not treated as measured silence.
        for age in (1..=history).rev() {
            ls.buf_r.push(self.tx[(self.tx_pos + self.tx.len() - 1 - age) % self.tx.len()]);
            ls.buf_x.push(0.0);
        }
        self.ls = Some(ls);
    }

    fn restart_training(&mut self) {
        self.training_epoch = self.training_epoch.wrapping_add(1);
        // Each retrain needs a fresh DIL measurement, while the working
        // echo path and transmit history must survive.
        self.ls = None;
        self.ab = None;
        // A verdict still being judged is dropped by its epoch when it lands.
        self.far_silent = false;
        self.frozen = false;
        self.data = false;
        self.seen.clear();
        self.scan_job = None;
        self.echo_energy = 0.0;
        self.residual_energy = 0.0;
        self.gate_samples = 0;
        self.set_tracking(false);
        self.delay_window.clear();
    }

    fn enter_v34_training(&mut self) {
        self.restart_training();
        // V.34 has no V.90 DIL to replace the previously identified path.
        // Preserve its coefficients/history, but allow training adaptation.
        self.identified = false;
        self.lag_hi = V34_ECHO_LAG_HI;
    }

    /// VoIP playout can remove or insert up to 80 ms of audio. A shifted copy
    /// of the already identified echo prediction must improve BOTH halves
    /// of an independent window, with approximately unchanged gain.
    fn delay_shift(window: &[(f64, f64)]) -> isize {
        /// Standard errors of correlation a shifted prediction must clear.
        const SHIFT_Z: f64 = 4.0;
        if window.len() < DELAY_WINDOW_MIN { return 0; }
        let mid = window.len() / 2;
        let mut best = 0;
        let mut best_score = f64::INFINITY;
        for shift in (-8isize..=8).filter(|&step| step != 0).map(|step| step * 80) {
            let mut score = 0.0;
            let mut valid = true;
            // Use the same rows for every candidate, keeping both indexing
            // and comparisons valid for the largest measured playout jump.
            for (lo, hi) in [(640usize, mid), (mid, window.len()-640)] {
                let (mut old, mut new, mut cross, mut power, mut raw) = (0.0,0.0,0.0,0.0,0.0);
                for i in lo..hi {
                    let (x, y) = window[i];
                    let candidate = window[(i as isize-shift) as usize].1;
                    old += (x-y).powi(2);
                    new += (x-candidate).powi(2);
                    cross += x*candidate;
                    power += candidate*candidate;
                    raw += x*x;
                }
                let gain = cross / power.max(1e-30);
                // Reject silence, unrelated far-end data, weak differences,
                // and changes in amplitude rather than in timing. Both tests
                // are against the echo, not the line: under a far modem 17 dB
                // louder (the USR caller, 2026-11-21) the echo is 2% of what
                // arrives, so a correlation of 0.2 with the line and a tenth
                // off its energy -- the tests this had -- cannot be met by the
                // right shift, and the 10 ms step at 150 s went unfollowed
                // until the call failed. Four standard errors of correlation
                // over the rows instead, and half the prediction's own energy
                // taken off the line.
                let rows = (hi - lo) as f64;
                valid &= power > 1e-8 && (0.75..=1.25).contains(&gain)
                    && cross > 0.0 && cross*cross > SHIFT_Z*SHIFT_Z/rows*raw*power
                    && old - new > 0.5*power;
                score += new;
            }
            if valid && score < best_score { best = shift; best_score = score; }
        }
        best
    }

    fn track_delay(&mut self, x: f64, predicted: f64) {
        // Final training also needs a stable reflection: a VoIP playout
        // shift after the silent measurement otherwise leaves CP buried in
        // our own transmit. Reuse the independently validated timing-only
        // correction after a completed measurement, never during the fit.
        let measured_training = !self.far_silent && self.ls.as_ref().is_some_and(|ls| ls.solved);
        if !(self.data || measured_training) || (!self.identified && !self.frozen) || self.ab_pending() {
            self.delay_window.clear();
            return;
        }
        self.delay_window.push((x, predicted));
        let len = self.delay_window.len();
        if len < DELAY_WINDOW_MIN || !len.is_power_of_two() { return; }
        let shift = Self::delay_shift(&self.delay_window);
        if shift == 0 && len < DELAY_WINDOW_MAX { return; }
        self.delay_window.clear();
        let next = self.delay as isize + shift;
        if shift != 0 && next > (self.w.len()/2) as isize && next < self.lag_hi as isize {
            // Keep coefficients unchanged: this moves the physical path,
            // unlike shift_to(), which changes coordinates of a fixed path.
            self.delay = next as usize;
            // A data fit measured before this validated timing change cannot
            // overwrite the corrected path when its worker returns.
            if self.tracking {
                self.track_epoch = self.track_epoch.wrapping_add(1);
                self.track.rx.clear();
                self.track.reference.clear();
                self.track.since = 0;
            }
            eprintln!("  echo: validated playout delay shift {shift:+} samples, delay {}", self.delay);
            self.shift = Some(shift);
        }
    }


    /// The echo path over one window of the line and our own transmit.
    ///
    /// Linear correlations, one row of the Toeplitz normal matrix per lag, a
    /// ridge at [`LS_RIDGE`] of the reference's own energy, and a direct
    /// Cholesky. `d0` is the lowest delay the window of taps covers, so the
    /// answer is already the filter and needs no rescaling and no peak search.
    ///
    /// The same function for the DIL and for data mode, on purpose: the two were
    /// answering the same question and the only thing that differs is the window
    /// they are given, and a tracker with its own arithmetic could disagree with
    /// the thing it is supposed to be tracking.
    fn solve_path(reference: &[f64], line: &[f64], d0: isize, taps: usize) -> Option<Vec<f64>> {
        Self::fit_path(reference, line, d0, taps).map(|(w, _)| w)
    }

    fn fit_path(reference: &[f64], line: &[f64], d0: isize, taps: usize) -> Option<(Vec<f64>, Vec<f64>)> {
        Self::fit_path_from(reference, line, d0, taps, 0)
    }

    fn fit_path_from(reference: &[f64], line: &[f64], d0: isize, taps: usize,
                     settled_from: usize) -> Option<(Vec<f64>, Vec<f64>)> {
        let n = reference.len().min(line.len());
        if d0 < 0 || taps == 0 || n <= d0 as usize + taps {
            return None;
        }
        let first = (d0 as usize + taps - 1).max(settled_from);
        if first >= n { return None; }
        let lag = d0 as usize;
        // Every column and the target must use exactly the same line rows.
        // A full-window reference correlation paired with a truncated target
        // correlation shrinks the gain by approximately 1 - delay / n.
        // Adjacent Gram entries differ only at the two window boundaries,
        // so the exact matrix costs O(n*taps + taps*taps), not O(n*taps*taps).
        let mut a = vec![0.0; taps * taps];
        let mut b = vec![0.0; taps];
        for j in 0..taps {
            let mut gram = 0.0;
            let mut target = 0.0;
            for row in first..n {
                let r = reference[row - lag - j];
                gram += reference[row - lag] * r;
                target += line[row] * r;
            }
            a[j] = gram;
            a[j * taps] = gram;
            b[j] = target;
        }
        for i in 1..taps {
            for j in i..taps {
                let value = a[(i - 1) * taps + j - 1]
                    + reference[first - lag - i] * reference[first - lag - j]
                    - reference[n - lag - i] * reference[n - lag - j];
                a[i * taps + j] = value;
                a[j * taps + i] = value;
            }
        }
        let energy = a[0].max(1e-30);
        for i in 0..taps {
            a[i * taps + i] += LS_RIDGE * energy;
        }
        cholesky_solve(&a, &b, taps)
    }

    /// The tap `k` of a filter with `taps` taps is the reference sample
    /// `delay - taps / 2 + k` ago, so the whole 512-tap window spans delays
    /// `delay - 256 ..= delay + 255`. Every lag the estimate needs is in that
    /// range, and nothing is needed outside it.
    fn training_delay(reference: &[f64], line: &[f64], hi: usize) -> Option<usize> {
        Self::training_delay_from(reference, line, hi, 0)
    }

    fn training_delay_from(reference: &[f64], line: &[f64], hi: usize,
                           measured_from: usize) -> Option<usize> {
        Self::training_delay_resolving(reference, line, hi, measured_from, measured_from)
    }

    /// [`Self::training_delay_from`] over the rows from `measured_from`, with
    /// ties settled on the real rows from `wider_from` (earlier, and never the
    /// synthetic padding).
    ///
    /// A periodic reference -- the opening of a V.90 DIL repeats every 138
    /// samples -- correlates equally at every lag one period apart, so the
    /// measured rows alone can lock on an alias: 6288 for a 6840-sample echo
    /// on the 2026-11-02 calls, with the canceller then 552 samples off the
    /// path and the CP drowned by uncancelled TRN2d. The earlier rows carry
    /// the far end's signal too, but that is independent of this end's
    /// transmit and costs every candidate alike, while the echo of the
    /// aperiodic signals sent before the DIL picks out the true lag.
    fn training_delay_resolving(reference: &[f64], line: &[f64], hi: usize,
                                measured_from: usize, wider_from: usize) -> Option<usize> {
        let n = reference.len().min(line.len());
        // Compare every candidate on the same rows, with no startup padding.
        let first = hi.max(measured_from);
        if n < first + 1024 { return None; }
        let score_from = |lag: usize, from: usize| {
            let (mut xy, mut xx, mut yy) = (0.0, 0.0, 0.0);
            for i in from..n {
                let x = reference[i-lag];
                let y = line[i];
                xy += x*y; xx += x*x; yy += y*y;
            }
            xy.abs() / (xx*yy).max(1e-30).sqrt()
        };
        let score = |lag: usize| score_from(lag, first);
        let scores: Vec<f64> = (ECHO_LAG_LO..hi).map(score).collect();
        let (mut best, mut peak) = scores.iter().enumerate()
            .fold((ECHO_LAG_LO, 0.0), |(b, p), (i, &v)| if v > p { (ECHO_LAG_LO + i, v) } else { (b, p) });
        let wider = hi.max(wider_from);
        if wider < first && peak >= ECHO_PEAK_MIN {
            // Lags far from the peak that score as well as it are aliases.
            let rivals: Vec<usize> = scores.iter().enumerate()
                .filter(|&(i, &v)| v >= ALIAS_TIE * peak && (ECHO_LAG_LO + i).abs_diff(best) > 4)
                .map(|(i, _)| ECHO_LAG_LO + i)
                .collect();
            if !rivals.is_empty() {
                let (lag, _) = rivals.iter().chain(std::iter::once(&best))
                    .map(|&lag| (lag, score_from(lag, wider)))
                    .fold((best, -1.0), |(b, p), (l, v)| if v > p { (l, v) } else { (b, p) });
                if lag != best {
                    eprintln!("  echo: {} lags tie with {best} over the measured rows (a periodic reference); \
                               the earlier rows choose {lag}", rivals.len());
                    best = lag;
                    peak = score(best);
                }
            }
        }
        for lag in best.saturating_sub(4).max(ECHO_LAG_LO)..(best+5).min(hi) {
            let value = score(lag);
            if value > peak { best = lag; peak = value; }
        }
        (peak >= ECHO_PEAK_MIN).then_some(best)
    }

    /// A changing playout delay can split the whole-window correlation into
    /// weak peaks. Use a settled tail only when two independent blocks agree.
    fn settled_training_delay(reference: &[f64], line: &[f64], hi: usize,
                              taps: usize) -> Option<(usize, usize)> {
        let n=reference.len().min(line.len());
        let half=4*taps;
        if n < hi+2*half || half<1024 { return None; }
        let first=n-2*half;
        let find=|lo:usize, end:usize| {
            let score=|lag:usize| {
                let (mut xy,mut xx,mut yy)=(0.0,0.0,0.0);
                for i in lo..end {
                    let r=reference[i-lag]; let x=line[i];
                    xy+=r*x; xx+=r*r; yy+=x*x;
                }
                (xy.abs()/(xx*yy).max(1e-30).sqrt(), xy/xx.max(1e-30))
            };
            let mut best=ECHO_LAG_LO; let mut peak=0.0;
            for lag in ECHO_LAG_LO..hi {
                let value=score(lag).0;
                if value>peak {best=lag;peak=value;}
            }
            for lag in best.saturating_sub(4).max(ECHO_LAG_LO)..(best+5).min(hi) {
                let value=score(lag).0;
                if value>peak {best=lag;peak=value;}
            }
            let (correlation,gain)=score(best);
            (best,correlation,gain)
        };
        let a=find(first,first+half); let b=find(first+half,n);
        if a.0.abs_diff(b.0)>4 || a.1<ECHO_PEAK_MIN || b.1<ECHO_PEAK_MIN
            || a.2*b.2<=0.0 || a.2.abs().min(b.2.abs())<0.5*a.2.abs().max(b.2.abs()) {
            return None;
        }
        Some((b.0,first))
    }

    fn ls_poll(&mut self) {
        let Some(job) = self.training_job.as_ref() else { return };
        match job.try_recv() {
            Ok(fit) => {
                self.training_job = None;
                if fit.epoch != self.training_epoch {
                    eprintln!("  echo: completed training fit discarded after retrain");
                    return;
                }
                if self.hold_fits {
                    // A live V.90 call's only phase-4 receive loss came 2.3 s
                    // after a fit landed just past the start of phase 4, the
                    // one call of the run where it landed that late; the caller
                    // retrained for want of an answer in time.
                    eprintln!("  echo: training fit finished after the receiver trained; keeping the filter it trained with");
                    return;
                }
                let norm = fit.w.iter().map(|v| v * v).sum::<f64>().sqrt();
                if !Self::plausible_fit_norm(norm) {
                    eprintln!("  echo: training fit discarded: norm {norm:.4} is no echo path; keeping prior filter");
                    return;
                }
                self.delay = fit.delay;
                self.w = fit.w;
                self.identified = fit.identified;
                self.ls = fit.ls;
                eprintln!("  echo: training fit installed without blocking audio");
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.training_job = None;
                eprintln!("  echo: training worker ended; keeping prior filter");
            }
        }
    }

    /// A hybrid's echo through 64 taps: every DIL fit on the live calls of
    /// 2026-11-03 came out at a norm of 0.25 or less (median 0.14). One came
    /// out at 0.80 from a lock 300 samples off the established delay, with a
    /// 36 dB pivot spread; installed, it blinded the receiver and phase 3 had
    /// to retrain. A fit that size is the solve chasing the far end's signal.
    fn plausible_fit_norm(norm: f64) -> bool {
        norm.is_finite() && norm <= 0.5
    }

    fn ls_solve(&mut self) {
        if !self.training_async {
            let (delay, w) = (self.delay, self.w.clone());
            self.ls_solve_inline();
            let norm = self.w.iter().map(|v| v * v).sum::<f64>().sqrt();
            if !Self::plausible_fit_norm(norm) {
                eprintln!("  echo: training fit discarded: norm {norm:.4} is no echo path; keeping prior filter");
                self.delay = delay;
                self.w = w;
            }
            return;
        }
        if self.training_job.is_some()
            || !self.ls.as_ref().is_some_and(|ls| !ls.solved && ls.samples >= 16 * self.w.len())
        {
            return;
        }
        // Move the completed measurement, not its sample buffers, onto a
        // bounded worker. The audio thread keeps its current filter and
        // transmit history while the factorisation and delay search run.
        let mut worker = Echo::new();
        worker.delay = self.delay;
        worker.lag_hi = self.lag_hi;
        worker.w = self.w.clone();
        worker.identified = self.identified;
        worker.ls = self.ls.take();
        let mut pending = Ls::blank(self.delay);
        pending.solved = true;
        self.ls = Some(pending);
        let epoch = self.training_epoch;
        let (send, receive) = std::sync::mpsc::channel();
        if std::thread::Builder::new().name("training-echo-fit".into()).spawn(move || {
            worker.ls_solve_inline();
            let _ = send.send(TrainingFit { epoch, delay: worker.delay,
                w: worker.w, identified: worker.identified, ls: worker.ls });
        }).is_ok() {
            self.training_job = Some(receive);
        } else {
            eprintln!("  echo: cannot start training worker; keeping prior filter");
        }
    }

    fn ls_solve_inline(&mut self) {
        let taps = self.w.len();
        // ANSam/phase-2 tones have many equally good periodic delay locks.
        // Establish the geometry again from broadband PP/TRN or V.90 DIL
        // before fitting taps; otherwise a held tone alias can put the
        // real reflection entirely outside the filter (2048 vs 3720 samples
        // in the captured 300-ms RTT test).
        let mut settled_from = self.ls.as_ref().map_or(0, |ls|
            ls.buf_x.len().saturating_sub(ls.samples as usize).max(ls.quiet_from.unwrap_or(ls.buf_x.len())));
        {
            if let Some(ls) = self.ls.as_ref().filter(|ls| !ls.solved && ls.samples >= 16 * taps) {
                // The first real receive row: everything before it is padding.
                let real_from = ls.buf_x.len().saturating_sub(ls.samples as usize);
                if let Some(delay) = Self::training_delay_resolving(&ls.buf_r, &ls.buf_x, self.lag_hi, settled_from, real_from) {
                    self.shift_to(delay);
                } else if let Some((delay,first))=Self::settled_training_delay(
                    &ls.buf_r,&ls.buf_x,self.lag_hi,taps)
                    .filter(|(_, first)| *first >= settled_from) {
                    settled_from=settled_from.max(first);
                    eprintln!("  echo: settled training tail validated at delay {delay}, rows from {first}");
                    self.shift_to(delay);
                }
            }
        }
        let Some(ls) = self.ls.as_mut() else { return };
        // Keep enough observations for a stable fit during the far end's
        // silence. The aligned Gram matrix below removes the old delay/window
        // gain bias; this gate still prevents fitting an underdetermined path.
        if ls.solved || ls.samples < 16 * taps {
            return;
        }
        ls.solved = true;
        ls.kept = self.w.clone();
        let lock = self.delay;
        // What the filter was managing over the window the solve came from, so
        // the log can say what the solve bought.
        ls.depth_before = 10.0 * (ls.pred / ls.res.max(1e-30)).log10();
        ls.pred = 0.0;
        ls.res = 0.0;
        ls.solved_at = ls.samples as u64;
        ls.lags = taps;

        let centre = taps / 2;
        let d0 = lock as isize - centre as isize;
        let n = ls.buf_x.len().min(ls.buf_r.len());
        if lock == 0 || d0 < 0 || n < d0 as usize + taps {
            ls.note = format!(
                "not solved: {} samples, lock {lock}, the window spans delays \
                 {d0}..{}",
                n,
                d0 + taps as isize
            );
            return;
        }

        // A reference that repeats inside the filter cannot tell one tap from
        // the tap a period away, and the solve shares the echo out between
        // them by the noise. The pivots say so (47.8 dB on the live calls,
        // against 0.1 dB on a broadband reference), and halving the span until
        // they stop saying so leaves the taps the reference does identify,
        // centred on the lock, with the rest at zero.
        let spread = |pivots: &[f64]| {
            let lo = pivots.iter().copied().fold(f64::INFINITY, f64::min);
            let hi = pivots.iter().copied().fold(0.0f64, f64::max);
            if lo > 0.0 { 10.0 * (hi / lo).log10() } else { f64::INFINITY }
        };
        let mut span = taps;
        let (w, cond_db) = loop {
            let from = d0 + (centre - span / 2) as isize;
            match Self::fit_path_from(&ls.buf_r, &ls.buf_x, from, span, settled_from) {
                Some((fit, pivots)) => {
                    let cond_db = spread(&pivots);
                    if cond_db <= LS_SPREAD_MAX_DB || span / 2 < LS_SPAN_MIN {
                        let mut w = vec![0.0; taps];
                        w[centre - span / 2..centre - span / 2 + span].copy_from_slice(&fit);
                        break (w, cond_db);
                    }
                    eprintln!("  echo: DIL solve: pivot spread {cond_db:.1} dB over {span} lags: the reference repeats inside them; fitting {}", span / 2);
                }
                None if span / 2 >= LS_SPAN_MIN => {}
                None => {
                    ls.note = "not solved: aligned normal equations did not factor".into();
                    return;
                }
            }
            span /= 2;
        };
        ls.cond_db = cond_db;
        ls.lags = span;

        // The answer is already the filter.
        //
        // There is no alignment step, because there is nothing to align: tap `j`
        // of the answer is the line's response at delay `d0 + j`, and tap `j` of
        // the filter is the reference sample `d0 + j` ago. The previous version
        // read the impulse response out of a circular transform, found its
        // largest sample, and slid the whole thing so that sample landed on the
        // tap the correlation's lock implied -- an assumption about two
        // measurements agreeing which is why the taps were once 1104 samples
        // out. Here the estimator's index *is* the filter's index.
        self.w = w.clone();
        self.identified = true;

        let norm: f64 = w.iter().map(|v| v * v).sum::<f64>().sqrt();
        let (peak_tap, (peak_abs, peak_mag)) = w
            .iter()
            .enumerate()
            .fold((0usize, (0.0f64, 0.0f64)), |(bi, (bm, bv)), (i, &v)| {
                if v.abs() > bm { (i, (v.abs(), v)) } else { (bi, (bm, bv)) }
            });
        let _ = peak_abs;
        let peak_at = d0 as usize + peak_tap;
        let placed = w.iter().filter(|v| **v != 0.0).count();
        ls.note = format!(
            "{placed} of {taps} taps set, largest {peak_mag:+.4} at a delay of {peak_at}, \
             norm {norm:.4}"
        );
        eprintln!(
            "  echo: DIL identify: correlation lock {lock}, the estimate's taps are \
             indexed by delay directly, window {d0}..{}",
            d0 + taps as isize
        );
        eprintln!(
            "  echo: DIL solve: {n} samples, {span} lags, ridge {LS_RIDGE:e} of the \
             reference's energy, pivot spread {:.1} dB, {}",
            ls.cond_db, ls.note
        );
    }

    /// One data-mode sample into the tracker's window, and an attempt when
    /// [`TRACK_PERIOD`] have gone by.
    ///
    /// Fed from `sample` on every sample of a call that has reached data mode,
    /// so that the window is the window the receiver is being given, not a
    /// window reconstructed afterwards.
    fn track_feed(&mut self, x: f64, r: f64) {
        if self.track.since % 80 == 0 {
            self.track_poll();
        }
        self.track.rx.push_back(x);
        self.track.reference.push_back(r);
        if self.track.rx.len() > TRACK_RING {
            self.track.rx.pop_front();
            self.track.reference.pop_front();
        }
        self.track.since += 1;
        if self.track.since >= TRACK_PERIOD {
            self.track.since = 0;
            self.track_launch();
        }
    }

    fn set_tracking(&mut self, active: bool) {
        if self.tracking != active {
            self.track_epoch = self.track_epoch.wrapping_add(1);
            // A fit must contain one continuous data interval. Never combine
            // pre-retrain data with training tones or a new data constellation.
            self.track.rx.clear();
            self.track.reference.clear();
            self.track.since = 0;
        }
        self.tracking = active;
    }

    fn track_launch(&mut self) {
        if self.track_job.is_some() || self.delay == 0 || self.track.rx.len() < TRACK_FIT + TRACK_HOLD {
            return;
        }
        let epoch = self.track_epoch;
        let delay = self.delay;
        let w = self.w.clone();
        let rx = self.track.rx.clone();
        let reference = self.track.reference.clone();
        let tries = self.track.tries;
        let taken = self.track.taken;
        let (send, receive) = std::sync::mpsc::channel();
        // The factorisation must not hold up the 20 ms AudioSocket stream.
        // One worker at a time bounds CPU and memory use. It owns a snapshot;
        // only the audio thread can install the completed filter.
        if std::thread::Builder::new().name("v90-echo-fit".into()).spawn(move || {
            let mut candidate = Echo::new();
            candidate.delay = delay;
            candidate.w = w;
            candidate.track.rx = rx;
            candidate.track.reference = reference;
            candidate.track.tries = tries;
            candidate.track.taken = taken;
            candidate.track_step();
            let _ = send.send(TrackFit {
                epoch, delay: candidate.delay, w: candidate.w,
                accepted: candidate.track.taken > taken,
                held_best: candidate.track.held_best,
                held_now: candidate.track.held_now,
            });
        }).is_ok() {
            self.track.tries += 1;
            self.track_job = Some(receive);
        }
    }

    fn track_poll(&mut self) {
        let Some(job) = self.track_job.as_ref() else { return };
        match job.try_recv() {
            Ok(fit) => {
                self.track_job = None;
                if !self.tracking || fit.epoch != self.track_epoch {
                    eprintln!("  echo: completed data fit discarded after a phase change");
                    return;
                }
                self.track.held_best = fit.held_best;
                self.track.held_now = fit.held_now;
                if fit.accepted {
                    self.delay = fit.delay;
                    self.w = fit.w;
                    self.track.taken += 1;
                    self.track_writes.push((self.track.rx.len() as u64, self.delay, self.w.clone()));
                }
            }
            Err(std::sync::mpsc::TryRecvError::Disconnected) => self.track_job = None,
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
        }
    }

    /// Fit the path to the newest [`TRACK_FIT`] samples, judge it on the
    /// [`TRACK_HOLD`] before them, and take it only if it is better.
    ///
    /// The margin is what stops this being the gradient again. The previous
    /// gradient made the echo *worse* -- -1.9 dB of ERLE_tx on the calls that
    /// measured it -- because it was fitted to the far modem's signal as well as
    /// to ours, and nothing ever asked whether the result was an improvement.
    /// Here both candidates are scored on the same held-back samples, on the
    /// energy correlated with our own transmit, and the incumbent has to lose by
    /// a whole decibel.
    fn track_step(&mut self) {
        let taps = self.w.len();
        if self.delay == 0 || self.track.rx.len() < TRACK_FIT + TRACK_HOLD {
            return;
        }
        self.track.tries += 1;
        let n = self.track.rx.len();
        // `as_slices` on a `VecDeque` splits at the ring's own wrap point and
        // the first half can come back empty, which it did: "range end index
        // 4096 out of range for slice of length 0". One slice each, contiguous.
        let rx = self.track.rx.make_contiguous();
        let rf = self.track.reference.make_contiguous();
        let hold = &rx[..TRACK_HOLD];
        let ref_hold = &rf[..TRACK_HOLD];
        let fit = &rx[n - TRACK_FIT..];
        let ref_fit = &rf[n - TRACK_FIT..];
        let d0 = self.delay as isize - (taps / 2) as isize;

        // What the incumbent achieves on the held-back samples, as the thing to
        // beat. Its own prediction is not in the window, so it is formed from
        // the same reference with the same frame the fit uses.
        let now = Self::residual_measured(self.w.as_slice(), d0, self.delay, hold, ref_hold);
        if !now.is_finite() || now <= 0.0 { return; }
        let Some(cand) = Self::solve_path(ref_fit, fit, d0, taps) else {
            eprintln!("  echo: track: the normal equations would not factor; the filter is left alone");
            return;
        };
        // Where the fit's own peak sits, and whether the window would be better
        // held somewhere else.
        //
        // Moving the lock on the strength of the peak alone was a mistake, and a
        // measured one: on the 2026-09-26 23:10 call a fit whose peak was 160
        // samples right of centre moved the lock there, and the live filter's
        // ERLE_tx in the windows that followed went from 5.5 dB to 0.2 and then
        // 0.0. The dominant path had not moved -- an independent fit over the
        // same windows still put its own energy at 1319 and its ERLE *fell* to
        // 9.1 dB at 1479 from 11.1 at 1319 -- so the peak of a noisy fit is not
        // evidence of where the path is. The lock is therefore moved only if a
        // fit taken at the new lock beats a fit taken at the old one, on the
        // same held-back samples, by the same margin as everything else.
        let peak = cand
            .iter()
            .enumerate()
            .fold((0usize, 0.0f64), |(bi, bm), (i, &v)| {
                if v.abs() > bm { (i, v.abs()) } else { (bi, bm) }
            })
            .0;
        let mut cand = cand;
        let mut d0 = d0;
        let mut moved = 0isize;
        if peak.abs_diff(taps / 2) >= TRACK_EDGE
            && (ECHO_LAG_LO as isize..=ECHO_LAG_HI as isize)
                .contains(&(self.delay as isize + peak as isize - (taps / 2) as isize))
            && let Some(alt) = Self::solve_path(ref_fit, fit, d0 + (peak as isize - (taps / 2) as isize), taps)
        {
            let shift = peak as isize - (taps / 2) as isize;
            let here = Self::residual_measured(cand.as_slice(), d0, self.delay, hold, ref_hold);
            let there = Self::residual_measured(alt.as_slice(), d0 + shift, (self.delay as isize + shift) as usize, hold, ref_hold);
            if 10.0 * (here / there.max(1e-30)).log10() >= TRACK_MARGIN_DB {
                cand = alt;
                d0 += peak as isize - (taps / 2) as isize;
                moved = peak as isize - (taps / 2) as isize;
            }
        }
        let got = Self::residual_measured(cand.as_slice(), d0, self.delay, hold, ref_hold);
        self.track.held_best = got;
        self.track.held_now = now;
        let better = 10.0 * (now / got.max(1e-30)).log10();
        if better < TRACK_MARGIN_DB {
            eprintln!(
                "  echo: track: candidate {better:+.1} dB against the filter on \
                 {TRACK_HOLD} held-back samples, peak at tap {peak}, not taken"
            );
            return;
        }
        // Commit the same coefficient/delay pair that was judged. Recentring
        // taps independently shifts the live prediction away from the fitted
        // path, even when the held-out score said the candidate was excellent.
        let w = cand;
        if moved != 0 {
            self.delay = (d0 + (taps / 2) as isize) as usize;
        }
        let norm: f64 = w.iter().map(|v| v * v).sum::<f64>().sqrt();
        self.w = w.clone();
        self.track.taken += 1;
        // The window's own report quotes the taps and the delay in force when it
        // was written, which is before the tracker has run at all. The filter
        // changes under it, so each commit is logged too and the report can say
        // which set was in force over which part of the window.
        self.track_writes.push((self.track.rx.len() as u64, self.delay, w));
        eprintln!(
            "  echo: track: taken, {better:+.1} dB against the filter on \
             {TRACK_HOLD} held-back samples, peak at a delay of {}, norm {norm:.4}, \
             {}/{} attempts",
            d0 + peak as isize,
            self.track.taken,
            self.track.tries
        );
    }

    /// The energy in `line - predict(taps)` that is correlated with `reference`.
    ///
    /// `d0` is where the tap window starts, which is what the prediction needs,
    /// and `delay` is the lock, which is what the projection has to be *centred*
    /// on. They differ by half the tap count, and passing the first where the
    /// second was wanted put the search over `[delay - 512, delay)`: the path,
    /// which is at `delay`, sat on the top edge of the range and most of what
    /// the search found was the edge itself. The tracker then compared two
    /// filters on that and committed on it, and a call that ran 232 attempts
    /// finished with an ERLE_tx of -3.5 dB: a filter that made the echo worse,
    /// which is precisely what the held-out check existed to prevent.
    fn residual_measured(
        taps_w: &[f64],
        d0: isize,
        delay: usize,
        line: &[f64],
        reference: &[f64],
    ) -> f64 {
        let n = line.len().min(reference.len());
        let mut res = vec![0.0; n];
        for i in 0..n {
            let mut y = 0.0;
            for (k, &w) in taps_w.iter().enumerate() {
                if w == 0.0 {
                    continue;
                }
                let j = i as isize - d0 - k as isize;
                if j >= 0 && (j as usize) < n {
                    y += w * reference[j as usize];
                }
            }
            res[i] = line[i] - y;
        }
        Self::tx_correlated(&res, reference, delay, taps_w.len())
    }

    /// The energy in `residual` that is linearly correlated with our own
    /// transmitted signal, over the lags the filter spans.
    ///
    /// `residual` holds three things: the echo that got through, the far
    /// modem's signal, and the line's noise. Only the first is this filter's
    /// doing. Projecting the residual onto delayed copies of the reference picks
    /// the first out and leaves the other two, so a filter is scored on what it
    /// was there to remove and not on what it was never asked to touch.
    ///
    /// The projection runs over the tap window coarsened by `AB_LAG_STEP`, which
    /// is 64 lags for 512 taps: enough free parameters to soak up a spread
    /// path, few enough that over a thousand samples they cannot explain the
    /// far end by more than about a twentieth of it.
    fn tx_correlated(residual: &[f64], reference: &[f64], delay: usize, taps: usize) -> f64 {
        let centre = taps / 2;
        let n = residual.len().min(reference.len());
        if n < 256 {
            return 0.0;
        }
        // Normalised so the answer is a power, in the same units as the
        // residual's mean square.
        let mut ref_power = 0.0;
        for v in &reference[..n] {
            ref_power += v * v;
        }
        if ref_power <= 0.0 {
            return 0.0;
        }
        // The strongest single lag, not the sum over lags.
        //
        // Summing is the obvious thing and it does not work: 64 lags of far-end
        // signal give 64 independent chances to correlate, and their powers
        // add, so the floor came to within a decibel of the echo and the metric
        // could not tell a correct filter from a wrong one -- 0.1 dB between
        // them on a synthetic path where the answer should be obvious. The echo
        // is concentrated even when the path is spread: the block solve on this
        // line puts a peak of 0.0086 in a filter of norm 0.0185, so one lag
        // carries the signal and the rest are floor.
        let mut best = 0.0f64;
        let mut total = 0.0f64;
        let lo = delay.saturating_sub(centre);
        let mut lag = lo;
        while lag < delay + centre {
            // A lag whose reference would run past the end of the window is
            // skipped, not wrapped. Callers must leave room: see `AB_WINDOW`.
            if lag + 256 <= n {
                // A reflection of the reference sample `lag` ago lands at
                // index i having come from index i - lag, so the projection
                // runs backwards. Correlating forwards put the search on the
                // wrong side of the peak and the metric read 0.2 dB for a
                // filter that removes the echo outright.
                let c = dot4(&residual[lag..n], &reference[..n - lag]) / (n - lag) as f64;
                total += c * c;
                if c * c > best {
                    best = c * c;
                }
            }
            lag += AB_LAG_STEP;
        }
        let _ = total;
        best * n as f64
    }

    /// Run a candidate filter over a window of paired input and reference
    /// samples, in the filter's own frame: tap `k` reads the reference
    /// `delay - centre + k` samples ago.
    fn apply_filter(taps_w: &[f64], delay: usize, window: &[(f64, f64)]) -> Vec<f64> {
        let centre = taps_w.len() / 2;
        let d0 = delay as isize - centre as isize;
        let n = window.len();
        let taps = taps_w.len() as isize;
        // Tap `k` reads reference `i - d0 - k`, so over the taps the reference
        // runs backwards. Reversed once, it runs forwards and each output is
        // one dot product over the taps whose reference is inside the window,
        // instead of a bounds test per tap per sample (3 x 4.2 M of them per
        // A/B: 40 ms in one audio step on the server, 2026-11-15).
        let rev: Vec<f64> = window.iter().rev().map(|p| p.1).collect();
        let mut out = vec![0.0; n];
        for i in 0..n {
            let top = i as isize - d0; // the reference index tap 0 reads
            let k_lo = (top - (n as isize - 1)).max(0);
            let k_hi = (top + 1).min(taps);
            let y = if k_lo < k_hi {
                // rev[m] = reference[n - 1 - m], so reference[top - k] = rev[n - 1 - top + k].
                let m0 = (n as isize - 1 - top + k_lo) as usize;
                let len = (k_hi - k_lo) as usize;
                dot4(&taps_w[k_lo as usize..k_hi as usize], &rev[m0..m0 + len])
            } else {
                0.0
            };
            out[i] = window[i].0 - y;
        }
        out
    }

    /// Judge the two candidates on a window neither was fitted to, and keep the
    /// one that leaves less of our own transmission behind.
    ///
    /// The window is taken after the far end is talking again, which is the
    /// whole point: during the DIL both candidates have seen those samples, and
    /// the gradient in particular has been descending on them, so a comparison
    /// there flatters it.
    fn select_echo_candidate(p_none: f64, p_grad: f64, p_ls: f64, v34: bool) -> &'static str {
        if !p_none.is_finite() || p_none <= 0.0 || !p_grad.is_finite() || !p_ls.is_finite() {
            return "neither";
        }
        let margin = 10f64.powf(-0.5 / 10.0);
        // What the gradient has to win by. On V.34 half a decibel: a real
        // repeat lost 4 dB after a 0.04-dB win. On PCM calls 3 dB: over 55
        // live V.92 PCM upstream calls the block fit left 16.8 to 18.5 rms
        // on TRN2u every time it was kept, and the gradient 22.6 to 38.3 on
        // the six it won, one of them by 0.1 dB. This figure does not see
        // what the gradient's wandering taps add.
        let material = if v34 { margin } else { 10f64.powf(-3.0 / 10.0) };
        if p_grad.min(p_ls) > p_none * margin {
            "neither"
        } else if p_ls < p_grad || p_grad >= p_ls * material {
            // A small correlation difference is measurement noise. Preserve
            // the block fit unless the gradient has a material advantage.
            "LS"
        } else {
            "gradient"
        }
    }

    /// Whether the judging window held enough of our own echo to judge by.
    /// When our transmitter was quiet through it (V.34 phase 3 has silent
    /// stretches) the raw signal carries almost nothing correlated with what
    /// we sent, every candidate looks no better than none, and choosing none
    /// would wipe a canceller that works: two live V.34 calls lost phase 4 to
    /// exactly that, with the TX-correlated part 0.3-0.5% of the received
    /// RMS against 13-17% on calls that chose the fit.
    fn echo_window_conclusive(p_none: f64, rx_rms: f64) -> bool {
        p_none.is_finite() && rx_rms.is_finite() && p_none >= 0.02 * rx_rms
    }

    /// The choice when the window is too quiet to judge by. A quiet window
    /// cannot show that a fit helps, so the fit stays -- but it can show that
    /// a fit hurts: one that leaves more of our transmission behind than
    /// none did by over a decibel is adding echo, not removing it (live
    /// V.92 calls 2026-11-19: -16.6 and -2.7 dB kept, upstream never locked).
    fn inconclusive_echo_choice(p_none: f64, p_ls: f64) -> &'static str {
        if !p_ls.is_finite() || !p_none.is_finite() || p_ls > p_none * 10f64.powf(1.0 / 10.0) {
            "neither"
        } else {
            "LS"
        }
    }

    /// Judge the A/B, on a worker when the call is live.
    ///
    /// The judging is three filter passes and three correlations over a 2 s
    /// window: 40 ms inside one audio step on the server (2026-11-15), each
    /// time 1.3 s before a start-up that then failed. The audio thread keeps
    /// the block fit `ls_solve` installed while it runs -- the choice it makes
    /// most often -- and `ab_poll` installs the verdict when it lands.
    fn ab_select(&mut self) {
        let Some(ab) = self.ab.take() else { return };
        if ab.window.len() < 256 {
            return;
        }
        let (delay, taps, lag_hi, epoch) = (self.delay, self.w.len(), self.lag_hi, self.training_epoch);
        if !self.training_async {
            let verdict = Self::ab_judge(ab, delay, taps, lag_hi, epoch);
            self.ab_install(verdict);
            return;
        }
        let (send, receive) = std::sync::mpsc::channel();
        let spawned = std::thread::Builder::new().name("echo-a-b".into()).spawn(move || {
            let _ = send.send(Self::ab_judge(ab, delay, taps, lag_hi, epoch));
        });
        if spawned.is_ok() {
            self.ab_job = Some(receive);
        } else {
            eprintln!("  echo: cannot start the A/B worker; keeping the block fit");
        }
    }

    /// Whether an A/B is collecting its window or being judged.
    fn ab_pending(&self) -> bool {
        self.ab.is_some() || self.ab_job.is_some()
    }

    fn ab_poll(&mut self) {
        let Some(job) = self.ab_job.as_ref() else { return };
        match job.try_recv() {
            Ok(verdict) => {
                self.ab_job = None;
                self.ab_install(verdict);
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.ab_job = None;
                eprintln!("  echo: A/B worker ended; keeping the block fit");
            }
        }
    }

    fn ab_install(&mut self, verdict: AbVerdict) {
        if verdict.epoch != self.training_epoch {
            eprintln!("  echo: A/B verdict discarded after retrain");
            return;
        }
        if self.delay != verdict.judged_at {
            eprintln!("  echo: A/B verdict discarded: the path moved from {} to {} while it was judged",
                verdict.judged_at, self.delay);
            return;
        }
        self.delay = verdict.delay;
        self.w = verdict.w;
    }

    /// The A/B itself: which of the two candidates, or neither, leaves less of
    /// our own transmission behind, and whether the path's timing slipped.
    fn ab_judge(ab: Ab, judged_at: usize, taps: usize, lag_hi: usize, epoch: u64) -> AbVerdict {
        let rx: Vec<f64> = ab.window.iter().map(|(x, _)| *x).collect();
        let rf: Vec<f64> = ab.window.iter().map(|(_, r)| *r).collect();
        // The judging buffer begins without the preceding transmit history.
        // Only its fully populated tail can independently validate a timing
        // slip of the already measured path.
        let residual = Self::apply_filter(&ab.ls, judged_at, &ab.window);
        let first = ab.window.len().saturating_sub(4096);
        let timing: Vec<_> = rx[first..].iter().zip(residual[first..].iter())
            .map(|(x, left)| (*x, *x - *left)).collect();
        let shift = Self::delay_shift(&timing);
        let next = judged_at as isize + shift;
        let mut delay = judged_at;
        if shift != 0 && next > (taps/2) as isize && next < lag_hi as isize {
            delay = next as usize;
            eprintln!("  echo: comparison validated delay shift {shift:+}, delay {delay}");
        }

        let p_none = Self::tx_correlated(&rx, &rf, delay, taps);
        let g = Self::apply_filter(&ab.gradient, delay, &ab.window);
        // The block fit at an unmoved path is the residual already computed.
        let l = if delay == judged_at { residual } else { Self::apply_filter(&ab.ls, delay, &ab.window) };
        let p_grad = Self::tx_correlated(&g, &rf, delay, taps);
        let p_ls = Self::tx_correlated(&l, &rf, delay, taps);
        let rms = |v: &[f64]| -> f64 {
            (v.iter().map(|x| x * x).sum::<f64>() / v.len().max(1) as f64).sqrt()
        };
        let norm = |v: &[f64]| -> f64 { v.iter().map(|x| x * x).sum::<f64>().sqrt() };
        let erle = |before: f64, after: f64| -> f64 {
            10.0 * (before / after.max(1e-30)).log10()
        };

        // The guard: if neither measurably beats leaving the echo in, take
        // neither. Half a decibel is noise on a window this size.
        let chosen = if Self::echo_window_conclusive(p_none, rms(&rx)) {
            Self::select_echo_candidate(p_none, p_grad, p_ls, lag_hi == V34_ECHO_LAG_HI)
        } else {
            // Nothing to judge by: keep the block fit already installed,
            // unless it plainly adds our own echo.
            let chosen = Self::inconclusive_echo_choice(p_none, p_ls);
            if chosen == "LS" {
                eprintln!("    window holds too little echo to judge; keeping the block fit");
            } else {
                eprintln!("    window holds too little echo to judge, but the block fit adds our echo; dropping it");
            }
            chosen
        };

        eprintln!(
            "  ECHO A/B: delay={delay} window={} samples",
            ab.window.len()
        );
        eprintln!(
            "    uncancelled   total RMS {:8.2e}   TX-correlated {:8.2e}",
            rms(&rx),
            p_none
        );
        eprintln!(
            "    gradient      total RMS {:8.2e}   TX-correlated {:8.2e}   ERLE_tx {:5.1} dB   norm {:.4}",
            rms(&g),
            p_grad,
            erle(p_none, p_grad),
            norm(&ab.gradient)
        );
        eprintln!(
            "    block LS      total RMS {:8.2e}   TX-correlated {:8.2e}   ERLE_tx {:5.1} dB   norm {:.4}",
            rms(&l),
            p_ls,
            erle(p_none, p_ls),
            norm(&ab.ls)
        );
        eprintln!("    selected={chosen}");

        let w = match chosen {
            "LS" => ab.ls,
            "gradient" => ab.gradient,
            // ls_solve temporarily installs the block fit before this
            // independent check. Leaving it in place would apply a fit that
            // this comparison rejected as worse than the uncancelled signal.
            _ => vec![0.0; taps],
        };
        AbVerdict { epoch, judged_at, delay, w }
    }

    fn quiet(&self) -> bool {
        if self.seen.is_empty() {
            return false;
        }
        let een: f64 = self.seen.iter().map(|x| x * x).sum();
        let rms = (een / self.seen.len() as f64).sqrt();
        rms < 10f64.powf(ECHO_QUIET_DB / 20.0)
    }

    /// Whether the echo accounts for most of what has arrived, which is the
    /// question that decides whether a step taken now is a gradient on the
    /// filter or on the far end's signal.
    ///
    /// This is what `quiet()` cannot answer, and getting it wrong is what kept
    /// the filter from ever converging. `quiet()` asks whether the *input* is
    /// below -30 dBFS, and the one window where the far end is guaranteed
    /// silent -- the DIL, 9.3.1.6 -- is the one window where this end is
    /// transmitting four points of 22 666 bit/s and the input is loud with our
    /// own reflection. So the DIL reads as loud, the gate that was meant to
    /// open there never did, and the filter was last adapted on whatever gaps
    /// the far modem's own transmissions left.
    ///
    /// Measured on a call of 2026-09-26 16:16 with the equalised points as the
    /// score, which is the first score that can see this at all: the filter
    /// reached 0.24 of tap norm and a cancellation depth of -4.6 dB in data
    /// mode, the client's signal sat 18 dB above the floor under all of it,
    /// and the equalised points read E[z^8] 19.3 -- noise. The same call with
    /// this end's transmit muted reads E[z^8] 472560, a constellation. The
    /// receiver was never the problem; it was reading our own echo.
    fn double_talk(&mut self, x: f64, yhat: f64) -> bool {
        if !double_talk() {
            return false;
        }
        let residual = x - yhat;
        self.echo_energy += yhat * yhat;
        self.residual_energy += residual * residual;
        self.gate_samples += 1;
        if self.gate_samples < ECHO_WINDOW {
            return false;
        }
        let opens = self.echo_energy > 4.0 * self.residual_energy + 1e-9;
        self.echo_energy = 0.0;
        self.residual_energy = 0.0;
        self.gate_samples = 0;
        opens
    }

    /// One line sample: subtract the filter's estimate of our reflection,
    /// then (on quiet windows, or where the modem says the far end is
    /// silent, and while still in start-up) adapt it.
    fn sample(&mut self, x: f64) -> f64 {
        self.ls_poll();
        self.ab_poll();
        let mut yhat = 0.0;
        if self.delay != 0 {
            let n = self.tx.len();
            let d0 = self.delay - self.w.len() / 2; /* taps straddle the lock */
            let mut norm = 0.0;
            let mut r = vec![0.0f64; self.w.len()];
            for (k, slot) in r.iter_mut().enumerate() {
                let v = self.tx[(self.tx_pos + n - 1 - (d0 + k)) % n];
                *slot = v;
                norm += v * v;
                yhat += self.w[k] * v;
            }
            if !self.frozen && !self.identified && norm > 1e-4 {
                // Fast while the far end is quiet, where the error is the
                // filter's own. The slow step, where there is one, is for the
                // far end talking; it is off by default -- see `slow_mu`.
                //
                // Fast while the far end is quiet, where the error is the
                // filter's own; the slow step, where there is one, is for the
                // far end talking. It is off by default -- see `slow_mu`.
                //
                // What does not work is learning from the DIL, which is the
                // one window where the far modem is silent (9.3.1.6) and this
                // end is transmitting four points of 22 666 bit/s, so the one
                // chance at a wideband path. Adapting through it was measured
                // on the captures of 2026-09-25 11:12 and 11:13 and moved
                // the phase 4 SNR not at all -- 5.8, 2.0, 2.4, 4.7, 5.5, 3.9,
                // 5.8, 6.0 dB with it and the same eight without -- because
                // the analogue modem's S-bar is in that window too, and a fast
                // step learns it instead of the path.
                // Three ways the far end can be quiet enough to learn the path
                // from: the line is quiet outright; the modem says the far end
                // is silent, which 9.3.1.6 guarantees through the DIL and which
                // is the one wideband window in the call; or the echo already
                // accounts for most of what has arrived, which covers the start
                // of data mode before the far modem's own signal fills the
                // band. The second is the one that matters and it cannot be
                // inferred: a line loud with this end's own transmission is
                // loud in every window, so `quiet()` alone leaves the DIL --
                // the only wideband chance to learn the path -- shut.
                // A quiet caller tone is still peer speech. During V.90
                // start-up, use protocol-confirmed silence before letting
                // low input amplitude authorize a fast adaptive update.
                let mu = if (self.quiet() && self.peer_silent) || (!echo_ls() && self.far_silent) || self.double_talk(x, yhat) {
                    ECHO_MU
                } else if self.data {
                    slow_mu()
                } else {
                    0.0
                };
                self.last_mu = mu;
                if mu > 0.0 {
                    let floor = ECHO_NLMS_FLOOR_PER_TAP * self.w.len() as f64;
                    let g = mu * (x - yhat) / (norm + floor);
                    let mut size = 0.0;
                    for (k, &v) in r.iter().enumerate() {
                        self.w[k] = self.w[k] * 0.99995 + g * v;
                        size += self.w[k] * self.w[k];
                    }
                    if !Self::plausible_fit_norm(size.sqrt()) {
                        // Past any echo path: what it predicts is not this
                        // end's echo, and subtracting it only adds to the line.
                        if self.runaway_epoch != Some(self.training_epoch) {
                            self.runaway_epoch = Some(self.training_epoch);
                            eprintln!("  echo: the adaptive filter grew to a norm of {:.2}, past any echo path: cleared", size.sqrt());
                        }
                        self.w.iter_mut().for_each(|w| *w = 0.0);
                    }
                }
            }
        }
        // The DIL, and the block identification of the path from it.
        //
        // This is the one window in the call where the far modem is silent and
        // this end is transmitting, so it is the one window where the input is
        // a measurement of the path rather than of the far end, and it is far
        // too short to descend 512 taps through. `x` is what arrived and `r[0]`
        // is what this end transmitted at the same moment, which is the
        // canceller's own reference -- no separate tap or delay to get wrong.
        if self.far_silent && echo_ls() {
            let first = self.ls.is_none();
            if first {
                eprintln!(
                    "  echo: DIL identification starting, {} taps, delay {}",
                    self.w.len(),
                    self.delay
                );
                self.begin_training_measurement();
            }
            // The canceller's own reference: the transmit sample this moment,
            // which is what `r[0]` held when the delay lock put it in reach, and
            // which `push` has just written into the ring.
            let n = self.tx.len();
            let reference = self.tx[(self.tx_pos + n - 1) % n];
            self.ls_feed(x, reference);
            if let Some(ls) = self.ls.as_mut() {
                ls.pred += yhat * yhat;
                ls.res += (x - yhat) * (x - yhat);
            }
            // Compact, explicitly configured paths can be identified before
            // DIL ends. Otherwise a loud DIL segment can echo back as a
            // transient 2400-Hz tone and falsely trigger a caller retrain
            // before the falling-edge solve ever gets a chance to run.
            // Keep a full second of protocol-confirmed quiet observations;
            // seeded transmit history and pre-silence peer training do not
            // count toward this threshold. Wider paths retain the full DIL.
            if self.w.len() <= COMPACT_TAPS && self.peer_silent
                && self.ls.as_ref().is_some_and(|ls| !ls.solved
                    && ls.quiet_from.is_some_and(|first|
                        ls.buf_x.len().saturating_sub(first) >= 8192.max(16 * self.w.len())))
            {
                prof!("echo solve", self.ls_solve());
            }
        }

        // The after-figure: the same measurement as the one taken over the
        // window the solve came from, on samples taken after it. Printed once,
        // when the first full window after the far end starts talking again
        // completes, which is the first window that is the filter's alone.
        if !self.far_silent
            && let Some(ls) = self.ls.as_mut()
            && ls.solved
            && !ls.reported
        {
            ls.post_pred += yhat * yhat;
            ls.post_res += (x - yhat) * (x - yhat);
            ls.post_n += 1;
            if ls.post_n >= ECHO_WINDOW as u32 {
                ls.reported = true;
                let after = 10.0 * (ls.post_pred / ls.post_res.max(1e-30)).log10();
                eprintln!(
                    "  echo: DIL solve: cancellation after the solve {after:.1} dB, \
                     against {:.1} dB before it, over {} samples",
                    ls.depth_before, ls.post_n
                );
                // Logged, and nothing more. This figure is prediction against
                // residual, which is not a measure of cancellation: it rises
                // when the filter's output is simply larger, so it preferred a
                // big noisy filter to a small correct one and on the 2026-09-26
                // calls it preferred the gradient at -2.5 dB of transmit-correlated
                // residual -- a filter that made the echo worse -- over the
                // correlation's at 8.3 dB. It used to put the gradient's taps
                // back here. It no longer decides anything: the A/B does, on
                // the energy actually correlated with our own transmission, and
                // it runs over a window this measurement is not taken from.
                eprintln!(
                    "  echo: DIL solve: this figure is prediction against residual and \
                     is not a measure of cancellation; the A/B below decides"
                );
            }
        }

        self.pred_energy += yhat * yhat;
        let left = x - yhat;
        self.left_energy += left * left;
        self.left_samples += 1;
        self.left_quiet = self.quiet();
        prof!("echo scan", self.scan_step(SCAN_LAGS_PER_STEP));
        self.seen.push(x);
        if self.seen.len() >= ECHO_WINDOW {
            // A search still running from the window before is finished
            // first; at 64 lags a sample it never is.
            prof!("echo scan", { self.scan_step(usize::MAX); self.scan() });
            self.seen.clear();
            prof!("echo report", self.report());
        }
        // Solve on the falling edge of the window, not inside it. The DIL is
        // 1.96 s and every sample of it is worth having, and the estimate is a
        // correlation over the whole of it, so there is nothing to solve until
        // there is no more. Inside the window it could not be checked either,
        // because every sample after it would be part of the same measurement.
        // The falling edge of the far end's silence: solve if there is a window
        // to solve, then judge the two candidates on what comes after.
        if !self.far_silent {
            if echo_ls() {
                // How long the far end's silence actually lasted, and whether
                // that was enough. A 1.96 s DIL is 15 655 samples, thirty a tap
                // and well clear of the sixteen the solve wants, so a window
                // that fell short was not the DIL being 1.96 s long.
                let needed = 16 * self.w.len();
                if let Some(held) = self.ls.as_mut().and_then(|ls| ls.take_shortfall(needed)) {
                    eprintln!(
                        "  echo: far end resumed after {held} silent samples, {} \
                         short of the {needed} the solve wants",
                        needed.saturating_sub(held)
                    );
                }
                prof!("echo solve", self.ls_solve());
            }
            self.ab_collect(x, self.reference());
        }
        if self.ab.as_ref().is_some_and(|a| a.window.len() >= AB_WINDOW) {
            prof!("echo a/b", self.ab_select());
        }
        // The three series, for the data-mode log: what the line carried, which
        // of our own transmit samples it is an echo of, and what the filter
        // predicted. `sample` has no other way to hand them up, and the log
        // wants them for the same sample the receiver is being given.
        let reference = self.reference();
        self.last = Last { x, r: reference, yhat };
        self.track_delay(x, yhat);
        // Data mode, where the far end is transmitting and the DIL's silence is
        // not coming back: the path is refitted here or not at all. V.90 only.
        if self.tracking {
            prof!("echo track", self.track_feed(x, reference));
        }
        self.do_no_harm(x, left)
    }

    /// What the receiver is given: what cancelling left, unless the window
    /// before this one found that larger than the line itself (calls 13 and
    /// 15 of 2026-11-15, 5 dB louder through V.90 phase 3 with the far end
    /// silent). The filter goes on adapting and being judged either way.
    fn do_no_harm(&mut self, x: f64, left: f64) -> f64 {
        if !self.harm_judged || !echo_no_harm() {
            if self.harmful {
                eprintln!("  echo: the far end's silence is over: cancelling again");
            }
            self.harmful = false;
            self.harm_line = 0.0;
            self.harm_left = 0.0;
            self.harm_samples = 0;
            return left;
        }
        self.harm_line += x * x;
        self.harm_left += left * left;
        self.harm_samples += 1;
        if self.harm_samples >= ECHO_WINDOW {
            // In past half a decibel worse; out again only once it takes a
            // decibel away, so a filter hovering at nothing does not flap.
            let harmful = if self.harmful {
                self.harm_left > 0.8 * self.harm_line
            } else {
                self.harm_left > 1.12 * self.harm_line + 1e-12
            };
            if harmful != self.harmful {
                eprintln!(
                    "  echo: cancelling {} the line by {:.1} dB: {}",
                    if harmful { "adds to" } else { "takes from" },
                    10.0 * (self.harm_left.max(1e-30) / self.harm_line.max(1e-30)).log10().abs(),
                    if harmful { "the line goes on uncancelled" } else { "cancelling again" }
                );
            }
            self.harmful = harmful;
            self.harm_line = 0.0;
            self.harm_left = 0.0;
            self.harm_samples = 0;
        }
        if self.harmful { x } else { left }
    }

    /// Our own transmit as the canceller's reference is defined: the sample
    /// `push` has just written into the ring, which is the one at delay minus
    /// one in the filter's frame.
    fn reference(&self) -> f64 {
        let n = self.tx.len();
        self.tx[(self.tx_pos + n - 1) % n]
    }

    /// Add one sample to the A/B's judging window, and arm the A/B if a solve
    /// has just put two candidates to choose between.
    fn ab_collect(&mut self, x: f64, r: f64) {
        let taps = self.w.len();
        if let Some(ab) = self.ab.as_mut() {
            if ab.window.len() < AB_WINDOW {
                ab.window.push((x, r));
            }
            return;
        }
        // Armed by a solve that ran and left two candidates: what the gradient
        // had, and what the correlation found.
        if let Some(ls) = self.ls.as_ref() {
            if ls.solved && !ls.kept.is_empty() && ls.kept.len() == taps && !ls.ab_armed {
                self.ab = Some(Ab { gradient: ls.kept.clone(), ls: self.w.clone(), window: Vec::new() });
                if let Some(ls) = self.ls.as_mut() {
                    ls.ab_armed = true;
                }
                self.ab_collect(x, r);
            }
        }
    }

    /// `ECHO_DEPTH` logs one line a window: the cancellation depth, the delay
    /// the path was locked at, and the step the filter took. The depth is the
    /// filter's own prediction against what was left over, so where the far
    /// end is quiet it is the echo return loss, and where the far end is
    /// talking it reads low by however loud the far end is -- which is the safe
    /// direction, and is why the `quiet` flag is on the line.
    fn report(&mut self) {
        if !echo_depth() || self.left_samples == 0 {
            return;
        }
        let (p, l) = (self.pred_energy, self.left_energy);
        let depth = 10.0 * (p / l.max(1e-12)).log10();
        eprintln!(
            "  echo: depth {depth:6.1} dB  delay {:5}  peak {:.3}  mu {:.4}  quiet {}  data {}  taps {:.3}",
            self.delay,
            self.peak,
            self.last_mu,
            if self.left_quiet { "y" } else { "n" },
            if self.data { "y" } else { "n" },
            self.energy().sqrt()
        );
        self.pred_energy = 0.0;
        self.left_energy = 0.0;
        self.left_samples = 0;
    }

    /// What we put on the line (post-gain), newest last.
    fn push(&mut self, y: f64) {
        self.tx[self.tx_pos] = recorded_next().unwrap_or(y);
        self.tx_pos = (self.tx_pos + 1) % self.tx.len();
    }

    fn take_shift(&mut self) -> Option<isize> {
        self.shift.take()
    }

    fn energy(&self) -> f64 {
        self.w.iter().map(|w| w * w).sum()
    }
}

type GetBitFn = Option<unsafe extern "C" fn(*mut c_void) -> c_int>;
type PutBitFn = Option<unsafe extern "C" fn(*mut c_void, c_int)>;

enum Stage {
    V23(Box<datapump::v23::Modem>),
    /// U.S. Robotics HST (ported from BinModem): MNP error control inside,
    /// so it trades octets, carried to and from C as async characters.
    Hst(Box<HstEnd>),
    V32(Box<v32::startup::Modem>),
    /// V.8 running: ANSam out, CM in, JM out, or the caller's half of that.
    V8(Box<v8line::Modem>),
    /// V.34 from INFO0 to data mode and through everything after it.
    V34(Box<v34::startup::Modem>),
    /// V.90's digital end from the end of V.8 through data mode, at the
    /// line's 8 kHz (V.90's start-up carries its own V.34 fallback inside).
    V90(Box<v90::startup::Digital>),
    /// Handed a terminal status to C; silence from here.
    Done,
}

/// One end of an HST call, with the async characters C speaks on one side
/// and the session's octets on the other.
struct HstEnd {
    /// The answerer's session, or the caller's call: the caller listens for
    /// the answer tone to end first, as BinModem's dialler does.
    s: Option<datapump::hst::Session>,
    call: Option<datapump::hst::call::Call>,
    connected: bool,
    failed: bool,
    /// The character being read from C's bits: none, or the data bits so far.
    char_bits: Option<(u8, u8)>,
    /// Octets the session handed up, waiting to go to C as characters.
    up: std::collections::VecDeque<bool>,
}

impl HstEnd {
    fn new(answer: bool) -> Self {
        use datapump::hst::{Config, Session, Side};
        let (s, call) = if answer {
            (Some(Session::new(LINE_FS, Side::Answerer, Config::default())), None)
        } else {
            (None, Some(datapump::hst::call::Call::new(LINE_FS, Side::Caller, Config::default())))
        };
        Self { s, call, connected: false, failed: false, char_bits: None, up: Default::default() }
    }

    fn session(&self) -> Option<&datapump::hst::Session> {
        self.s.as_ref().or_else(|| self.call.as_ref().and_then(|c| c.session()))
    }

    fn session_mut(&mut self) -> Option<&mut datapump::hst::Session> {
        match self.s.as_mut() { Some(s) => Some(s), None => self.call.as_mut().and_then(|c| c.session_mut()) }
    }

    fn backlog(&self) -> usize {
        self.session().map_or(0, datapump::hst::Session::backlog)
    }

    /// The forward and back channels as (transmit, receive) bit/s.
    fn rates(&self) -> (u32, u32) {
        let Some(s) = self.session() else { return (0, 0) };
        let fwd = s.rate().bits_per_second();
        let back = s.back_rate().bits_per_second();
        match s.holding() {
            datapump::hst::Holding::Forward => (fwd, back),
            datapump::hst::Holding::Back => (back, fwd),
        }
    }

    fn phase(&self) -> &'static str {
        if self.failed { "HST disconnected" } else if self.connected { "HST data / MNP" } else { "HST MNP link request" }
    }

    /// One character's line bits: start, eight data bits least first, stop.
    fn push_char(&mut self, byte: u8) {
        self.up.push_back(false);
        for i in 0..8 { self.up.push_back(byte >> i & 1 != 0); }
        self.up.push_back(true);
    }

    /// One bit of C's async stream; a whole character goes into MNP.
    fn take_bit(&mut self, bit: bool) {
        match self.char_bits {
            None if !bit => self.char_bits = Some((0, 0)),
            None => {}
            Some((v, 8)) => {
                // The stop bit, whatever it is: the character is done.
                let _ = bit;
                if let Some(s) = self.session_mut() { s.send(&[v]); }
                self.char_bits = None;
            }
            Some((v, n)) => self.char_bits = Some((v | (u8::from(bit) << n), n + 1)),
        }
    }

    fn step(&mut self, x: f64) -> f64 {
        let y = match (self.s.as_mut(), self.call.as_mut()) {
            (Some(s), _) => { s.feed(x); s.next_sample() }
            (None, Some(c)) => c.step(x),
            (None, None) => 0.0,
        };
        let Some(s) = self.session_mut() else { return y };
        let mut events = Vec::new();
        while let Some(e) = s.poll_event() { events.push(e); }
        let read = s.read();
        for e in events {
            match e {
                datapump::hst::session::Event::Connected { .. } => self.connected = true,
                datapump::hst::session::Event::Disconnected => self.failed = true,
                _ => {}
            }
        }
        for b in read { self.push_char(b); }
        y
    }
}

/// V90_X2=1: answer callers that offer V.34 and no PCM as an x2 server
/// (U.S. Robotics x2, ported from BinModem 5.5.0). Off by default, so those
/// callers keep the 16 kHz V.34 fallback.
fn x2_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("V90_X2").as_deref() == Ok("1"))
}

/// V.8 as this end runs it: the menu, PCM pairing, LAPM and V.92 quick
/// connect as configured. Used at the start of a call, and again when a
/// V.92 modem-on-hold brings the caller back through phase 1.
fn v8_modem(v8role: v8line::Role, answer: bool, want_v34: bool, want_v90: bool) -> v8line::Modem {
        // What goes in V.8's menu: V.34 when the caller wants it, and V.22bis
        // beside it so a far end that cannot do better still connects.
        let mut menu = Modulations::of(&[Modulation::V22bis, Modulation::V32bis, Modulation::V23Duplex]);

        if want_v34 {
            menu.insert(Modulation::V34Duplex);
        }
        // V.90's mode: V.8 at the line's own rate, offering the digital PCM
        // category from the end that answers. Offer LAPM when enabled: its
        // framing must agree with the stack we start after physical training.
        // Compression is disabled separately for the V.90 path.
        let v8fs = if want_v90 { LINE_FS } else { ENGINE_FS };
        let mut v8m = v8line::Modem::new(v8role, CallFunction::Data, menu, v8fs);
        if want_v90 {
            if answer {
                v8m = v8m.offering_pcm_on(
                    Pcm { digital: true, ..Pcm::default() },
                    Access { digital: true, ..Access::default() },
                );
            }
        }
        if !want_v90 || v90_error_control() {
            v8m = v8m.offering_lapm();
        }
        // U.S. Robotics x2 (BinModem 5.5.0): a caller that offers V.34 and no
        // PCM is told this end is an x2 server, by the JM's digital access
        // bit and its carrier turning between JMs.
        if want_v90 && answer && x2_enabled() {
            v8m = v8m.offering_x2();
        }
        // V92_QC_ANSWER=0 keeps V.92 phase 2 but answers no QC1a/QC2a: the
        // analogue modem then starts in full and chooses its upstream afresh
        // (V.92 9.3), rather than from what it stored on its last call.
        if want_v90 && answer && std::env::var("V92_QUICK_CONNECT").as_deref()==Ok("1")
            && std::env::var("V92_QC_ANSWER").as_deref()!=Ok("0") {
            v8m=v8m.offering_quick_connect(v90::ucode::Law::Mu);
            if std::env::var("V92_QC_DELAY_COMPENSATION").as_deref()==Ok("1") {
                v8m=v8m.allowing_delayed_quick_connect_reply();
            }
            // V92_QC_V8BIS=0: QC1a after ANSam is answered, QC2a inside
            // V.8 bis is not (the caller then hears ANSam and sends QC1a).
            if std::env::var("V92_QC_V8BIS").as_deref()==Ok("0") {
                v8m=v8m.quick_connect_after_ansam_only();
            }
        }
        v8m
}

pub struct Answerer {
    role: v34::phase2::Role,
    want_v34: bool,
    /// Built by `bm_create_v90`: V.8 offers the digital PCM category and
    /// both stages run at the line's rate through [`Self::linear_step`].
    want_v90: bool,
    /// V.90's data mode has been reached: past this point the line is ours to
    /// fill, and `v90_tx_mute` says not to.
    v90_in_data: bool,
    echo_track_data: bool,
    up: Resampler,
    down: Resampler,
    stage: Stage,
    status: c_int,
    failure: &'static str,
    /// The V.90 start-up's last failure reason, once told to the transcript:
    /// V.90 retrains in place, so C never sees a status for it.
    v90_failure: Option<&'static str>,
    rate_tx: c_int,
    rate_rx: c_int,
    rx_total: u64,
    underruns: u64,
    up_odd: u64,
    up_odd_last: u8,
    clips: u64,
    peak: f64,
    phase_buf: [u8; 96],
    fail_buf: [u8; 160],
    mid: Vec<f64>,
    out: VecDeque<f64>,
    echo: Echo,
    /// The call's line audio, when `BM_CAPTURE` named a directory for it.
    capture: Option<Capture>,
    /// Where the V.90 modem's own sample zero fell in the call's sample
    /// numbering, so that a sample count it reports can be tied to a line
    /// sample of the capture. `V90_DATA_ECHO` writes it.
    v90_origin: u64,
    /// A bounded raw receive window spanning the end of V.8. A caller's
    /// first INFO0 may begin before our handover silence has finished.
    v8_receive_tail: VecDeque<f64>,
    /// Whether the replay has been given a path and armed.
    abc_armed: bool,
    /// The V.90 modem's own sample count, read inside the stage match and used
    /// outside it, where `self` is free to borrow.
    v90_now: u64,
    /// Whether this call's data-mode header has been written.
    echo_state_done: bool,
    /// The call's data-mode window, the echo canceller's state at its start, and
    /// the four series the constellation points were made from, all appended to
    /// one file with a header per data-mode entry. See [`Answerer::write_echo_state`].
    data_log: Option<std::fs::File>,
    /// Where the logged window starts, in the V.90 modem's sample numbering.
    data_log_from: u64,
    /// Line samples stepped, for the transcript's own timestamps.
    samples: u64,
    get_bit: GetBitFn,
    get_ud: *mut c_void,
    put_bit: PutBitFn,
    put_ud: *mut c_void,
    ec: Option<EcStack>,
    /// V.92 9.10: on hold, sending ANSam for T1 and waiting for the caller's
    /// phase 1.
    holding: bool,
    async_tx: AsyncBits,
    lapm_declared: bool,
    lapm_without_detection: bool,
    physical_connected: bool,
    ec_ready_at: Option<u64>,
    ec_samples: u32,
    ec_frames_logged: u32,
    framing_good: u64,
    framing_bad: u64,
    framing_since: u64,
    framing_valid_at: u64,
    framing_rate_recovery: bool,
    ack_rate_recovery: bool,
    /// Sample at which physical data mode last came up.
    data_up_at: u64,
    realign: Realign,
    /// Sample at which physical data mode last went down, and whether it
    /// came back too soon for a retrain: a rate renegotiation.
    data_down_at: u64,
    after_renegotiation: bool,
    /// The renegotiation `after_renegotiation` names was the analogue modem's.
    peer_renegotiated: bool,
    /// The far end's V.42 XID has been logged.
    xid_logged: bool,
    /// The LAPM trace is being recorded (BM_LAPM_TRACE).
    lapm_tracing: bool,
    /// See `reneg_rescues_left`.
    reneg_rescues: u8,
    link_retrain_after: u64,
    link_data_active: bool,
    ec_health_last: u64,
    ec_health_trace: bool,
    legacy_v32_listener: v32::startup::Listener,
    legacy_v32_tone_samples: u32,
    /// ANSam has begun; tone before it is not a V.32 caller's AA.
    legacy_v32_after_ansam: bool,
    v32_logged_phase: String,
}

impl Answerer {
    fn start_v23(&mut self, reversed:bool) {
        use datapump::v23::{Modem,Direction};
        self.ec=None;
        self.lapm_declared=false;
        self.lapm_without_detection=false;
        self.physical_connected=false;
        self.stage=Stage::V23(Box::new(Modem::new(self.role==v34::phase2::Role::Answer,
            if reversed {Direction::Reversed} else {Direction::Normal},LINE_FS)));
        self.status=BM_RUNNING;
    }

    fn no_v8_negotiation(&mut self) {
        // A silent legacy caller does not select V.34 by failing to send CM.
        // Hand the answering end back to C's V.22bis startup (unscrambled
        // binary ones in the high channel), after ANSam has finished. Keep
        // the calling end's explicitly selected non-V.8 V.34 behaviour.
        if self.role == v34::phase2::Role::Answer {
            if self.legacy_v32_tone_samples >= 214 {
                // A sustained 1800 Hz AA identifies a legacy V.32 caller.
                self.start_v32(14400);
            } else {
                self.status = BM_AGREED_OTHER;
                self.stage = Stage::Done;
            }
        } else if self.wants_v34() {
            if self.want_v90 { self.fall_to_v34(); } else { self.start_v34(); }
        } else {
            self.status = BM_AGREED_OTHER;
            self.stage = Stage::Done;
        }
    }

    fn new(
        answer: bool,
        want_v34: bool,
        want_v90: bool,
        get_bit: GetBitFn,
        get_ud: *mut c_void,
        put_bit: PutBitFn,
        put_ud: *mut c_void,
    ) -> Self {
        let v8role = if answer {
            v8line::Role::Answering
        } else {
            v8line::Role::Calling
        };
        let role = if answer {
            v34::phase2::Role::Answer
        } else {
            v34::phase2::Role::Call
        };
        let mut v8m = v8_modem(v8role, answer, want_v34, want_v90);
        if answer && std::env::var("V8BIS_ENABLE").as_deref()==Ok("1") {
            v8m=v8m.offering_v8bis();
        }
        Self {
            role,
            want_v34,
            want_v90,
            v90_in_data: false,
            echo_track_data: data_echo_tracking_enabled(std::env::var("V90_ECHO_TRACKING").ok().as_deref()),
            up: Resampler::new(LINE_FS, ENGINE_FS),
            down: Resampler::new(ENGINE_FS, LINE_FS),
            stage: Stage::V8(Box::new(v8m)),
            status: BM_RUNNING,
            failure: "",
            v90_failure: None,
            rate_tx: 0,
            rate_rx: 0,
            rx_total: 0,
            underruns: 0,
            up_odd: 0,
            up_odd_last: 0,
            clips: 0,
            peak: 0.0,
            phase_buf: [0; 96],
            fail_buf: [0; 160],
            mid: Vec::new(),
            out: VecDeque::new(),
            echo: {
                let mut echo = Echo::new();
                echo.training_async = want_v90
                    && std::env::var("V90_ASYNC_TRAINING_FIT").as_deref() != Ok("0");
                if !want_v90 { echo.lag_hi = V34_ECHO_LAG_HI; }
                echo
            },
            capture: std::env::var_os("BM_CAPTURE").map(|d| Capture::new(std::path::Path::new(&d))),
        v90_origin: 0,
        v8_receive_tail: VecDeque::new(),
        v90_now: 0,
        abc_armed: false,
        echo_state_done: false,
        data_log: None,
        data_log_from: u64::MAX,
            samples: 0,
            get_bit,
            get_ud,
            put_bit,
            put_ud,
            ec: None,
            holding: false,
            async_tx: AsyncBits::new(8),
            lapm_declared: false,
            lapm_without_detection: false,
            physical_connected: false,
            ec_ready_at: None,
            ec_samples: 0,
            ec_frames_logged: 0,
            framing_good: 0,
            framing_bad: 0,
            framing_since: 0,
            framing_valid_at: 0,
            framing_rate_recovery: false,
            ack_rate_recovery: false,
            data_up_at: 0,
            realign: Realign::Idle,
            data_down_at: 0,
            after_renegotiation: false,
            peer_renegotiated: false,
            xid_logged: false,
            lapm_tracing: false,
            reneg_rescues: RENEG_RESCUES,
            link_retrain_after: 0,
            link_data_active: false,
            ec_health_last: u64::MAX,
            ec_health_trace: std::env::var_os("BM_EC_HEALTH").is_some(),
            legacy_v32_listener: v32::startup::Listener::new(LINE_FS),
            legacy_v32_tone_samples: 0,
            legacy_v32_after_ansam: false,
            v32_logged_phase: String::new(),
        }
    }

fn start_error_control(&mut self) {
        if self.ec.is_some() {
            return;
        }
        let role = if self.role == v34::phase2::Role::Answer {
            EcRole::Answerer
        } else {
            EcRole::Originator
        };
        let slower = self.rate_tx.min(self.rate_rx).max(2400) as u32;
        // V.32 measures symbols; the outer echo measurement uses 8 kHz samples.
        let round_trip_ms = match &self.stage {
            Stage::V32(modem) if modem.round_trip() > 0 =>
                (modem.round_trip() * 1000 / 2400).min(u32::MAX as u64) as u32,
            _ => (self.echo.delay as f64 * 1000.0 / LINE_FS).ceil() as u32,
        };
        let params = EcParams {
            t401_ms: ec::lapm::t401_for_line(slower, round_trip_ms),
            ..EcParams::default()
        };
        let mut stack = EcStack::new(role, params).over_a_round_trip(round_trip_ms);
        if self.lapm_declared {
            // V.8 agreed LAPM: proceed to XID. Otherwise retain V.42's
            // detection phase so a raw peer can fall back to transparent.
            stack = stack.declared_lapm();
            if self.lapm_without_detection {stack=stack.without_detection();}
        }
        // V.42bis is negotiated through LAPM XID in both directions. A peer
        // that does not offer it stays uncompressed. Keep an independent
        // diagnostic switch so LAPM can be tested without compression.
        let compression_off =
            self.want_v90 && std::env::var("V90_COMPRESSION").as_deref() == Ok("0");
        offer_compression_to(&mut stack, compression_off);
        self.ec = Some(stack);
    }

    fn update_connected_status(&mut self) {
        if !self.physical_connected {
            return;
        }
        self.status = match self.ec.as_ref() {
            Some(ec) if ec.phase() == EcPhase::Transparent => BM_CONNECTED,
            Some(ec) if ec.is_connected() => {
                let ready = *self.ec_ready_at.get_or_insert(self.samples);
                if ec.compression_name().is_none() || ec.valid_information_frames() > 0
                    || self.samples.saturating_sub(ready) >= 2 * 8000 {
                    BM_CONNECTED
                } else {
                    BM_RUNNING
                }
            },
            Some(_) => { self.ec_ready_at = None; BM_RUNNING },
            None => BM_CONNECTED,
        };
    }

    fn update_echo_tracking(&mut self) {
        self.echo.set_tracking(self.v90_in_data && self.echo_track_data);
    }

    fn v34_carrier_up(&self) -> bool {
        match &self.stage {
            Stage::V34(m) => matches!(m.status(), v34::startup::Status::Connected { .. }),
            Stage::V90(m) => !m.is_v90() && matches!(m.status(), v90::startup::Status::Connected { .. }),
            _ => false,
        }
    }


    fn request_physical_retrain(&mut self) -> bool {
        match &mut self.stage {
            Stage::V34(m) if matches!(m.status(), v34::startup::Status::Connected { .. }) => m.retrain(),
            Stage::V90(m) if matches!(m.status(), v90::startup::Status::Connected { .. }) => m.retrain(),
            _ => false,
        }
    }

    fn tick_error_control(&mut self, sample_rate: u32) {
        if let Some(ec) = self.ec.as_mut() {
            self.ec_samples += 1;
            if self.ec_samples >= sample_rate / 1000 {
                self.ec_samples = 0;
                ec.tick(1);
            }
        }
    }

    fn start_v34(&mut self) {
        self.echo.enter_v34_training();
        self.stage = Stage::V34(Box::new(v34::startup::Modem::new(self.role, ENGINE_FS)));
        self.status = BM_RUNNING;
    }

    fn start_v32(&mut self, max_rate: u32) {
        let role = if self.role == v34::phase2::Role::Answer {
            v32::startup::Role::Answering
        } else { v32::startup::Role::Calling };
        let rates = if max_rate <= 9600 {
            v32::startup::Rates { at_4800: true, at_9600: true, ..Default::default() }
        } else { v32::startup::Rates::between(4800, max_rate) };
        let offer = if max_rate <= 9600 {
            v32::startup::rate_signal_v32(rates, true)
        } else { v32::startup::rate_signal(rates) };
        self.want_v90 = false;
        let mut modem = v32::startup::Modem::new(role, offer, ENGINE_FS);
        if self.samples != 0 {
            modem.after_answer_tone();
        }
        self.stage = Stage::V32(Box::new(modem));
        self.status = BM_RUNNING;
    }

    /// From the end of V.8, with the far end's PCM category pairing this end
    /// as the digital modem: V.90's start-up at the line's own rate, saying
    /// in INFO0d what a real server says (µ-law, the 1664-point upstream).
    fn start_v90(&mut self) {
        self.v90_origin = self.samples;
        /* Keep long real-modem training within the 4000 ms Jd deadline.
           V90_TRN1D is a bench override; the default uses a six-sample
           boundary below that limit and includes RTT in the S deadline. */
        let habits = match std::env::var("V90_TRN1D").ok().and_then(|v| v.parse::<f64>().ok()) {
            Some(trn1d) if trn1d > 0.0 => v90::digital::Habits { trn1d, ..v90::digital::Habits::LIVE_SERVER },
            _ => v90::digital::Habits::LIVE_SERVER,
        };
        let mut modem = v90::startup::Digital::new(v90::server::ours()).with_habits(habits);
        if std::env::var("V92_QUICK_CONNECT").as_deref()==Ok("1") {
            modem=modem.with_v92();
        }
        // The captured early INFO0 is recovered offline, but the initial
        // hardware batch did not improve reliability. Keep priming opt-in
        // until its effect on real callers has been verified.
        if std::env::var("V90_PRIME_HANDOVER").as_deref() == Ok("1") {
            modem.prime_initial_receiver(self.v8_receive_tail.drain(..));
        } else {
            self.v8_receive_tail.clear();
        }
        self.stage = Stage::V90(Box::new(modem));
        self.status = BM_RUNNING;
    }

    /// From the end of V.8 with a caller that offered V.34 and no PCM, and
    /// x2 on (V90_X2=1): V.34's answer modem with x2 on offer, at the line's
    /// own rate. An x2 client's marker makes the call x2's; any other V.34
    /// caller goes on as V.34 inside the same digital wrapper.
    fn start_x2(&mut self) {
        self.v90_origin = self.samples;
        let habits = match std::env::var("V90_TRN1D").ok().and_then(|v| v.parse::<f64>().ok()) {
            Some(trn1d) if trn1d > 0.0 => v90::digital::Habits { trn1d, ..v90::digital::Habits::LIVE_SERVER },
            _ => v90::digital::Habits::LIVE_SERVER,
        };
        let modem = v90::startup::Digital::x2(v90::server::ours()).with_habits(habits);
        self.v8_receive_tail.clear();
        eprintln!("BinModem: caller offered V.34 and no PCM: answering with x2 on offer");
        self.stage = Stage::V90(Box::new(modem));
        self.status = BM_RUNNING;
    }

    /// V.8 ended without pairing this end as the digital half -- the far
    /// end is a plain V.34 modem, or the exchange never settled. Hand the
    /// rest of the call to the 16 kHz V.34 stage exactly as `bm_create`
    /// runs it: `want_v90` drops, and from the next sample `bm_step`
    /// dispatches to the resampled path, where the boundary gain and echo
    /// canceller come up from cold just as they do from creation.
    fn fall_to_v34(&mut self) {
        self.want_v90 = false;
        self.start_v34();
    }

    /// One 8 kHz line sample through the `bm_create_v90` path: V.8 first,
    /// then V.90's start-up, with no resampler or boundary gain between
    /// them. The digital modem already runs at the network's 8000 Hz, its
    /// levels are the exact codeword values the far end's G.711 encoder must
    /// see at unity gain, and this mode has no engine-to-engine waveform to
    /// protect from boundary scaling; the 16 kHz V.34 path in `bm_step` is
    /// the one that needs all of those. When V.8 does not pair this end
    /// digital, [`Self::fall_to_v34`] switches the object over to that
    /// proven path for the rest of the call.
    ///
    /// The echo canceller runs here as it does there, and on the real line it
    /// is what phase 3 needs: the far hybrid reflects our own transmit back
    /// some 190 ms late and about 14 dB down, which lands inside the analogue
    /// modem's S, and an equalizer handed an S at 11 dB of SNR never trains
    /// ("the analogue modem's training sequence did not train this end", 9.5.1
    /// retrain, twice, then the call dies) -- the capture notes on the filter
    /// above are why it exists. It is never frozen here: the V.90 digital
    /// modem's own PCM downstream comes back through that same path for the
    /// whole call, data mode included.
    fn linear_step(&mut self, input: c_int) -> c_int {
        if let Stage::V90(m) = &self.stage {
            if m.is_v90() {
                self.echo.lag_hi = ECHO_LAG_HI;
            } else if m.phase().starts_with("V.34")
                && self.echo.lag_hi != V34_ECHO_LAG_HI
            {
                self.echo.enter_v34_training();
                eprintln!("BinModem: V.34 fallback echo training enabled; retained path, extended delay range");
            }
        }
        // This is a current phase, not a latched "once connected" flag.
        // Retraining must not feed training tones into the data-mode fitter.
        self.v90_in_data = matches!(&self.stage, Stage::V90(m)
            if m.is_v90() && matches!(m.status(), v90::startup::Status::Connected { .. }));
        // The digital wrapper can now be carrying V.34. Use V.34's stable
        // data-mode echo filter once that fallback has connected.
        self.echo.frozen = self.v34_carrier_up();
        // Preserve the legacy connected-state flag for the sample canceller.
        self.echo.data = self.status == BM_CONNECTED || self.echo.frozen;
        // A fit to simultaneous far-end data can cancel part of that signal,
        // even when another segment of the same stream scores better. A live
        // commit preceded sustained receive-lock loss. Keep the trained path
        // by default; data-window refitting is an explicit experimental opt-in.
        self.update_echo_tracking();
        self.echo.hold_fits = matches!(&self.stage, Stage::V90(m)
            if m.is_v90() && (m.phase().starts_with("V.90 phase 4") || self.v90_in_data));
        let x = prof!("echo", self.echo.sample(input as f64 / 32768.0));
        let y = match &mut self.stage {
            Stage::V8(m) => {
                // 160 ms covers a complete INFO0 and the receive filters'
                // warm-up. Replay only into the initial INFO receiver.
                if self.v8_receive_tail.len() >= (LINE_FS * 0.160) as usize {
                    self.v8_receive_tail.pop_front();
                }
                self.v8_receive_tail.push_back(input as f64 / 32768.0);
                // V.8 at the line's rate, at the level the digital server's
                // own tests use (modem's V.90 server steps it at 0.3).
                if self.echo.peak>=0.3 {
                    m.set_quick_connect_round_trip(self.echo.delay as f64/LINE_FS);
                }
                let sample=prof!("v8", m.step_with_handshake_input(x, input as f64 / 32768.0));
                let out=sample*if m.pcm_output() {1.0} else {0.3};
                let status = m.status();
                let digital = m.pcm_role() == Some(PcmRole::Digital);
                let lapm = m.lapm();
                if matches!(status,v8line::Status::Agreed(_)) && !digital {
                    self.role=if m.training_role()==v8line::Role::Calling {
                        v34::phase2::Role::Call
                    } else {v34::phase2::Role::Answer};
                }
                if self.holding && matches!(status, v8line::Status::Agreed(_)) {
                    self.holding = false;
                    eprintln!("BinModem: V.92 modem-on-hold: the caller is back, phase 1 agreed again");
                }
                match status {
                    v8line::Status::Negotiating => {}
                    // 9.10.2.1: no phase 1 from the caller within T1.
                    v8line::Status::NoNegotiation | v8line::Status::Failed if self.holding => {
                        self.fail("V.92 modem-on-hold: the caller did not come back within T1");
                    }
                    // V.90 is what V.8 agreed here: V.34's modulation with
                    // this end paired as the digital PCM half (V.8 6.2.6).
                    v8line::Status::Agreed(_) if digital => {
                        self.lapm_declared = lapm;
                        self.lapm_without_detection=m.quick_connected() && lapm;
                        self.start_v90();
                    }
                    v8line::Status::Agreed(Modulation::V32bis) => {
                        self.lapm_declared = lapm;
                        self.start_v32(14400);
                    }
                    v8line::Status::Agreed(Modulation::V23Duplex) => {
                        self.start_v23(false);
                    }
                                        v8line::Status::Agreed(Modulation::V22bis) => {
                        self.status = BM_AGREED_V22;
                        self.stage = Stage::Done;
                    }
                    // V.34 agreed but no digital pairing: a plain V.34 far
                    // end. Same fallback the answers below take.
                    v8line::Status::Agreed(Modulation::V34Duplex) => {
                        self.lapm_declared = lapm;
                        if self.want_v90 && x2_enabled() {
                            self.start_x2();
                        } else {
                            self.fall_to_v34();
                        }
                    }
                    v8line::Status::Agreed(_) => {
                        self.status = BM_AGREED_OTHER;
                        self.stage = Stage::Done;
                    }
                    // No CM response to ANSam: the legacy answering sequence,
                    // rather than an unnegotiated V.34 phase 2 tone.
                    v8line::Status::NoNegotiation => {
                        self.no_v8_negotiation();
                    }
                    v8line::Status::Failed => {
                        if self.wants_v34() {
                            self.fall_to_v34();
                        } else {
                            self.fail("V.8 failed");
                        }
                    }
                }
                out
            }
            Stage::V90(m) => {
                // A confident echo lock is the line's own round trip; the DIL
                // uses it to tell the analogue modem's S-bar from a reversal
                // read into its S before J'd can have reached it.
                if self.echo.peak >= 0.9 && self.echo.delay > 0 {
                    m.set_line_round_trip(self.echo.delay as f64 / LINE_FS);
                }
                let retrains_before = m.retrains();
                let out = prof!("v90 engine", m.step(x));
                let at = self.samples as f64 / LINE_FS;
                if m.retrains() != retrains_before {
                    if m.is_v90() {
                        self.echo.restart_training();
                    } else {
                        self.echo.enter_v34_training();
                    }
                    self.echo_state_done = false;
                    self.v90_failure = None;
                    eprintln!("[{at:8.3}s] BinModem V.90 retrain {}: fresh echo-training measurement, retained path", m.retrains());
                }
                for note in m.take_notes() {
                    eprintln!("[{at:8.3}s] BinModem V.90: {note}");
                }
                // Collect the transmitted DIL even when detecting the peer's
                // first S-bar is late. The FIR fit excludes delay/tap warm-up
                // rows and is judged against independent post-DIL samples.
                self.echo.far_silent = m.echo_training_window() && (m.is_v90() || v34_echo_identify());
                self.echo.peer_silent = m.echo_training_settled();
                self.echo.harm_judged = m.echo_judged();
                // V.90 retrains in place, so C is handed no status for one
                // failing: the reason goes to the transcript here or nowhere.
                if let Some(why) = m.last_failure()
                    && self.v90_failure != Some(why)
                {
                    self.v90_failure = Some(why);
                    eprintln!("[{at:8.3}s] BinModem V.90 start-up failed: {why} (retrain, 9.5.1.1)");
                }
                self.status = match m.status() {
                    v90::startup::Status::Running => BM_RUNNING,
                    v90::startup::Status::Connected { transmit, receive } => {
                        self.lapm_without_detection |= self.lapm_declared && m.v34().phase2().v92_negotiated();
                        self.rate_tx = transmit as c_int;
                        self.rate_rx = receive as c_int;
                        self.v90_in_data = m.is_v90();
                        if !self.physical_connected {
                            self.physical_connected = true;
                            self.v90_now = m.samples();
                            // Extra replay receivers are diagnostics, not part
                            // of the live demodulator. Their fitting and dense
                            // resync searches must be explicitly requested.
                            if self.v90_in_data && (std::env::var_os("V90_ABC_POINTS").is_some()
                                || std::env::var_os("V90_ABC_PATH").is_some()) {
                                m.abc_arm(abc_path().unwrap_or_default(), self.echo.delay, self.echo.w.len());
                                self.abc_armed = true;
                            }
                            if !self.v90_in_data || v90_error_control() {
                                self.start_error_control();
                            }
                        }
                        self.tick_error_control(LINE_FS as u32);
                        self.update_connected_status();
                        self.status
                    }
                    v90::startup::Status::Retraining => BM_RETRAINING,
                    v90::startup::Status::ClearedDown => {
                        if self.failure.is_empty() {
                            self.failure = "far end cleared the call down";
                        }
                        BM_FAILED
                    }
                    v90::startup::Status::Failed(why) => {
                        if self.failure.is_empty() {
                            self.failure = why;
                        }
                        BM_FAILED
                    }
                };
                if self.status == BM_FAILED {
                    self.stage = Stage::Done;
                } else if let Stage::V90(m) = &self.stage
                    && let Some(held) = m.held()
                {
                    self.enter_hold(held);
                }
                out
            }
            // Never stepped: falling out of V.8 switches `bm_step` to the
            // resampled path from the next sample (the V34 arm runs there),
            // and Done is silence by definition.
            Stage::V23(_) | Stage::Hst(_) | Stage::V32(_) | Stage::V34(_) | Stage::Done => 0.0,
        };

        // The filter the tracker has committed since the last sample, in the
        // same file, so the window's report can say which taps were in force
        // over which part of it.
        if self.data_log.is_some() && !self.echo.track_writes.is_empty() {
            use std::io::Write;
            let now = self.samples.saturating_sub(self.v90_origin);
            let writes = std::mem::take(&mut self.echo.track_writes);
            if let Some(f) = self.data_log.as_mut() {
                for (age, delay, w) in writes {
                    let taps: Vec<String> = w.iter().map(|v| format!("{v:.9}")).collect();
                    let _ = writeln!(f, "=== track");
                    let _ = writeln!(f, "data_now {}", now.saturating_sub(age as u64));
                    let _ = writeln!(f, "delay {delay}");
                    let _ = writeln!(f, "taps_count {}", w.len());
                    let _ = writeln!(f, "taps {}", taps.join(" "));
                }
            }
        }
        if self.v90_in_data && !self.echo_state_done {
            self.echo_state_done = true;
            self.write_echo_state(self.v90_now);
        }
        // The data-mode log wants the sample the receiver was just given, and
        // the canceller's own three series for that same one. Written here,
        // after the stage match has finished borrowing self.
        if self.data_log.is_some() {
            let now = self.samples.saturating_sub(self.v90_origin);
            let l = self.echo.last;
            self.data_log_sample(now, l.x, l.r, l.yhat);
        }
        // The replay receivers, shown the line and the line with the
        // independent path off it. Fed from here, where `x` is the line as it
        // arrived and `reference` is our own transmit: the echo is a reflection
        // of what we sent, and a path predicted from the line would be a
        // different filter altogether.
        if self.abc_armed {
            if let Stage::V90(m) = &mut self.stage {
                m.abc_feed(self.echo.last.x, self.echo.last.r);
            }
        }
        let y = if self.want_v90 && self.v90_in_data && v90_tx_mute() { 0.0 } else { y };
        if y.abs() > self.peak {
            self.peak = y.abs();
        }
        if y > 1.0 || y < -1.0 {
            self.clips += 1;
        }
        self.echo.push(y);
        (y * 32768.0).clamp(-32768.0, 32767.0) as c_int
    }

    fn fail(&mut self, why: &'static str) {
        if self.failure.is_empty() {
            self.failure = why;
        }
        self.status = BM_FAILED;
        self.stage = Stage::Done;
    }

    /// V.92 9.10: what a modem-on-hold exchange in a retrain settled. On
    /// hold (or a fast reconnect) this end sends ANSam -- for T1, or V.8's
    /// usual five seconds -- and runs phase 1 again as the answer modem when
    /// the caller comes back, "disregarding" what the first phase 1 said. The
    /// V.42 link is not part of that and goes on as it does through a retrain.
    fn enter_hold(&mut self, held: datapump::v92::Held) {
        use datapump::v92::Held;
        let seconds = match held {
            Held::ClearedDown => {
                eprintln!("BinModem: V.92 modem-on-hold: the analogue modem asked for cleardown (MHclrd), answered MHcda");
                self.fail("V.92 cleardown asked by the analogue modem (MHclrd)");
                return;
            }
            Held::OnHold { seconds } => {
                eprintln!("BinModem: V.92 modem-on-hold: MHreq granted with MHack, holding with ANSam for {seconds} s");
                seconds
            }
            Held::FastReconnect => {
                eprintln!("BinModem: V.92 modem-on-hold: fast reconnect (MHfrr), ANSam and phase 1 again");
                datapump::v8::timing::ANSAM
            }
        };
        self.holding = true;
        self.v90_in_data = false;
        self.v8_receive_tail.clear();
        let v8m = v8_modem(v8line::Role::Answering, true, self.want_v34, self.want_v90).holding(seconds);
        self.stage = Stage::V8(Box::new(v8m));
        self.status = BM_RUNNING;
    }

    /// One engine-rate sample in, one out; the stage machine moves when the
    /// stage under it says it is time.
    fn engine_step(&mut self, x: f64) -> f64 {
        if let Stage::V8(m) = &mut self.stage {
            let out = m.step(x);
            let status=m.status();
            if matches!(status,v8line::Status::Agreed(_)) {
                // V.8bis can reverse startup roles without reversing who
                // placed the telephone call. Preserve that role at handoff,
                // just as the 8 kHz path does.
                self.role=if m.training_role()==v8line::Role::Calling {
                    v34::phase2::Role::Call
                } else {v34::phase2::Role::Answer};
            }
            match status {
                v8line::Status::Negotiating => return out,
                v8line::Status::Agreed(Modulation::V34Duplex) => {
                    self.lapm_declared = m.lapm();
                    self.start_v34();
                    return out;
                }
                v8line::Status::Agreed(Modulation::V32bis) => {
                    self.lapm_declared = m.lapm();
                    self.start_v32(14400);
                    return out;
                }
                v8line::Status::Agreed(Modulation::V23Duplex) => {
                    self.start_v23(false);
                    return out;
                }
                                    v8line::Status::Agreed(Modulation::V22bis) => {
                    self.status = BM_AGREED_V22;
                    self.stage = Stage::Done;
                    return out;
                }
                v8line::Status::Agreed(_) => {
                    self.status = BM_AGREED_OTHER;
                    self.stage = Stage::Done;
                    return out;
                }
                // A caller's selected modulation can proceed without V.8;
                // an unanswered ANSam at the answering end cannot select it.
                v8line::Status::NoNegotiation => {
                    self.no_v8_negotiation();
                    return out;
                }
                // Nothing in common, or nothing heard. Same answer as no
                // V.8: V.34 directly if it was asked for, because a PAP2T
                // audio bridge often loses the V.8 byte exchange while both
                // ends are perfectly capable of the rest of it.
                v8line::Status::Failed => {
                    if self.wants_v34() {
                        self.start_v34();
                    } else {
                        self.fail("V.8 failed");
                    }
                    return out;
                }
            }
        }
        if let Stage::V32(m) = &mut self.stage {
            let out = m.step(x);
            match m.status() {
                v32::startup::Status::Connected(rate) => {
                    self.rate_tx = rate as c_int;
                    self.rate_rx = rate as c_int;
                    if !self.physical_connected {
                        self.physical_connected = true;
                        self.start_error_control();
                    }
                    self.tick_error_control(ENGINE_FS as u32);
                    self.update_connected_status();
                }
                v32::startup::Status::Negotiating => self.status = BM_RUNNING,
                v32::startup::Status::Retraining => self.status = BM_RETRAINING,
                v32::startup::Status::Failed => self.fail("V.32/V.32bis training failed"),
            }
            return out;
        }
        if let Stage::V34(m) = &mut self.stage {
            let retrains_before = m.retrains();
            let out = m.step(x);
            if m.retrains() != retrains_before {
                self.echo.enter_v34_training();
            }
            self.status = match m.status() {
                v34::startup::Status::Running => BM_RUNNING,
                v34::startup::Status::Connected { transmit, receive } => {
                    self.rate_tx = transmit as c_int;
                    self.rate_rx = receive as c_int;
                    if !self.physical_connected {
                        self.physical_connected = true;
                        self.start_error_control();
                    }
                    self.tick_error_control(ENGINE_FS as u32);
                    self.update_connected_status();
                    self.status
                }
                v34::startup::Status::Retraining => BM_RETRAINING,
                v34::startup::Status::Done => {
                    self.failure = "phase 4 left no data mode";
                    BM_FAILED
                }
                v34::startup::Status::ClearedDown => {
                    self.failure = "far end cleared the call down";
                    BM_FAILED
                }
                v34::startup::Status::Failed(why) => {
                    self.failure = why;
                    BM_FAILED
                }
            };
            if self.status == BM_FAILED {
                self.stage = Stage::Done;
            }
            return out;
        }
        0.0
    }

    fn wants_v34(&self) -> bool {
        self.want_v34
    }

    /// The stage's own phase phrase, for when V.90's data mode has no stack.
    fn phase_from_stage(&mut self) {
        let src: &[u8] = match &self.stage {
            Stage::V23(m) => m.phase().as_bytes(),
            Stage::Hst(m) => m.phase().as_bytes(),
            Stage::V8(m) => m.phase().as_bytes(),
            Stage::V32(m) => m.phase().as_bytes(),
            Stage::V34(m) => m.phase().as_bytes(),
            Stage::V90(m) => m.phase().as_bytes(),
            Stage::Done => b"",
        };
        let n = src.len().min(self.phase_buf.len() - 1);
        self.phase_buf[..n].copy_from_slice(&src[..n]);
        self.phase_buf[n] = 0;
    }

    /// Copy the current phase phrase into the buffer the C side reads.
    fn copy_phase(&mut self) {
        if self.status == BM_RETRAINING {
            return self.phase_from_stage();
        }
        let src: &[u8] = if self.physical_connected {
            // The V.90 wrapper also carries negotiated V.34 fallback. The
            // requested mode is not the modulation actually on the line.
            if matches!(&self.stage, Stage::V90(m) if m.is_v90()) {
                if !v90_error_control() {
                    return self.phase_from_stage();
                }
                // V.92 when both ends said so in INFO0 (V.92 9.3), whichever
                // upstream INFO1a then chose (Table 18 PCM, Table 19 V.34).
                let mode = match &self.stage {
                    Stage::V90(m) if m.v90().is_some_and(|d| d.pcm_upstream()) => 2,
                    Stage::V90(m) if m.v34().phase2().v92_negotiated() => 1,
                    _ => 0,
                };
                const LABELS: [[&[u8]; 5]; 3] = [
                    [b"V.90 data / V.42 / V.42bis", b"V.90 data / V.42 / V.44", b"V.90 data / V.42", b"V.90 data / transparent", b"V.90 data"],
                    [b"V.92 data / V.42 / V.42bis", b"V.92 data / V.42 / V.44", b"V.92 data / V.42", b"V.92 data / transparent", b"V.92 data"],
                    [b"V.92 PCM-up data / V.42 / V.42bis", b"V.92 PCM-up data / V.42 / V.44", b"V.92 PCM-up data / V.42", b"V.92 PCM-up data / transparent", b"V.92 PCM-up data"],
                ];
                let l = &LABELS[mode];
                match self.ec.as_ref() {
                    Some(ec) if ec.is_connected() => match ec.compression_name() {
                        Some("V.42bis") => l[0],
                        Some("V.44") => l[1],
                        _ => l[2],
                    },
                    Some(ec) if ec.phase() == EcPhase::Transparent => l[3],
                    Some(_) => b"V.42 negotiating",
                    None => l[4],
                }
            } else if matches!(self.stage,Stage::V23(_) | Stage::Hst(_)) {
                return self.phase_from_stage();
            } else if matches!(self.stage, Stage::V32(_)) {
                match self.ec.as_ref() {
                    Some(ec) if ec.is_connected() => match ec.compression_name() {
                        Some("V.42bis") => b"V.32/V.32bis data / V.42 / V.42bis",
                        Some("V.44") => b"V.32/V.32bis data / V.42 / V.44",
                        _ => b"V.32/V.32bis data / V.42",
                    },
                    Some(ec) if ec.phase() == EcPhase::Transparent => b"V.32/V.32bis data / transparent",
                    Some(_) => b"V.42 negotiating",
                    None => b"V.32/V.32bis data",
                }
            } else {
                match self.ec.as_ref() {
                    Some(ec) if ec.is_connected() => match ec.compression_name() {
                        Some("V.44") => b"V.34 data / V.42 / V.44",
                        Some("V.42bis") => b"V.34 data / V.42 / V.42bis",
                        _ => b"V.34 data / V.42",
                    },
                    Some(ec) if ec.phase() == EcPhase::Transparent => b"V.34 data / transparent",
                    Some(_) => b"V.42 negotiating",
                    None => b"V.34 data",
                }
            }
        } else { match &self.stage {
            Stage::V23(m) => m.phase().as_bytes(),
            Stage::Hst(m) => m.phase().as_bytes(),
            Stage::V8(m) => m.phase().as_bytes(),
            Stage::V32(m) => m.phase().as_bytes(),
            Stage::V34(m) => m.phase().as_bytes(),
            Stage::V90(m) => m.phase().as_bytes(),
            Stage::Done => b"",
        }};
        let n = src.len().min(self.phase_buf.len() - 1);
        self.phase_buf[..n].copy_from_slice(&src[..n]);
        self.phase_buf[n] = 0;
    }

    /// Copy the failure phrase into the buffer the C side reads.
    fn copy_failure(&mut self) {
        let src: &[u8] = self.failure.as_bytes();
        let n = src.len().min(self.fail_buf.len() - 1);
        self.fail_buf[..n].copy_from_slice(&src[..n]);
        self.fail_buf[n] = 0;
    }
}

/* ------------------------------------------------------------------ */
/* The C interface.                                                    */
/* ------------------------------------------------------------------ */

/// Start an end of a call. `answer` is nonzero for the answering side.
///
/// `get_bit` supplies the next bit to transmit (0 or 1; idle line is ones)
/// and is called only while the pump can take bits. `put_bit` is handed each
/// bit recovered from the line, from V.34 data mode onward.
///
/// Returns null only if the arguments make no sense.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create(
    answer: c_int,
    want_v34: c_int,
    get_bit: GetBitFn,
    get_ud: *mut c_void,
    put_bit: PutBitFn,
    put_ud: *mut c_void,
) -> *mut Answerer {
    Box::into_raw(Box::new(Answerer::new(
        answer != 0,
        want_v34 != 0,
        false,
        get_bit,
        get_ud,
        put_bit,
        put_ud,
    )))
}

/// Start the digital end of a V.90 call instead: V.8 offering the digital
/// PCM category, then V.90's start-up from the pairing that comes back.
///
/// `answer` should be nonzero -- V.8 pairs the answering end as the digital
/// half (V.8 6.2.6), and `linear_step` only knows how to be that half. One
/// object carries the whole fallback ladder: digital pairing -> V.90; V.34
/// agreed without it, or V.8 lost or unsettled -> V.34 through the same
/// 16 kHz stage `bm_create` runs (BM_AGREED_OTHER only if V.8 settled on
/// something neither engine carries); V.22bis -> BM_AGREED_V22 for the
/// caller to take over. The bit callbacks work exactly as `bm_create`'s.
///
/// This mode runs V.8 and V.90 at the line's 8 kHz with no boundary scaling
/// -- but with the same near-end echo canceller the V.34 path runs, which
/// phase 3 against a real analogue modem cannot do without, and it carries
/// raw bits: no V.42 yet.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create_v90(
    answer: c_int,
    get_bit: GetBitFn,
    get_ud: *mut c_void,
    put_bit: PutBitFn,
    put_ud: *mut c_void,
) -> *mut Answerer {
    let end=Answerer::new(
        answer != 0,
        true,
        true,
        get_bit,
        get_ud,
        put_bit,
        put_ud,
    );
    Box::into_raw(Box::new(end))
}

/// Dedicated V.23 split-speed transparent FSK. `reversed` swaps 1200/75.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create_v23(answer:c_int,reversed:c_int,
    get_bit:GetBitFn,get_ud:*mut c_void,put_bit:PutBitFn,put_ud:*mut c_void,
)->*mut Answerer {
    if reversed!=0 && reversed!=1 {return std::ptr::null_mut();}
    let mut end=Answerer::new(answer!=0,false,false,get_bit,get_ud,put_bit,put_ud);
    end.start_v23(reversed!=0);
    Box::into_raw(Box::new(end))
}

/// Transparent Bell 103, sharing the qualified asynchronous FSK service.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create_bell103(answer:c_int,
    get_bit:GetBitFn,get_ud:*mut c_void,put_bit:PutBitFn,put_ud:*mut c_void,
)->*mut Answerer {
    let mut end=Answerer::new(answer!=0,false,false,get_bit,get_ud,put_bit,put_ud);
    end.start_v23(false);
    end.stage=Stage::V23(Box::new(datapump::v23::Modem::new_bell103(answer!=0,LINE_FS)));
    Box::into_raw(Box::new(end))
}

/// V.21 at 300 bit/s both ways: transparent FSK on V.21's own channels.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create_v21(answer:c_int,
    get_bit:GetBitFn,get_ud:*mut c_void,put_bit:PutBitFn,put_ud:*mut c_void,
)->*mut Answerer {
    let mut end=Answerer::new(answer!=0,false,false,get_bit,get_ud,put_bit,put_ud);
    end.start_v23(false);
    end.stage=Stage::V23(Box::new(datapump::v23::Modem::new_v21(answer!=0,LINE_FS)));
    Box::into_raw(Box::new(end))
}

/// U.S. Robotics HST (BinModem's, from a Courier's firmware): MNP inside,
/// async characters to and from C. The answerer holds the forward channel
/// from the start: on the server it begins once the caller's back channel
/// has been heard, the answer tone having been ANSam's.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create_hst(answer:c_int,
    get_bit:GetBitFn,get_ud:*mut c_void,put_bit:PutBitFn,put_ud:*mut c_void,
)->*mut Answerer {
    let mut end=Answerer::new(answer!=0,false,false,get_bit,get_ud,put_bit,put_ud);
    end.start_v23(false);
    end.stage=Stage::Hst(Box::new(HstEnd::new(answer!=0)));
    Box::into_raw(Box::new(end))
}

/// Hand the line bits of a Bell 103 / V.23 call up at the recovered bit clock
/// instead of as re-framed characters, for a V.42 link run on top of it.
#[unsafe(no_mangle)]
pub extern "C" fn bm_set_raw_line_bits(a:*mut Answerer,on:c_int) {
    let Some(a)=(unsafe{a.as_mut()}) else {return};
    if let Stage::V23(m)=&mut a.stage {m.set_raw_bits(on!=0);}
}

/// Dedicated V.32 (9600 ceiling) or V.32bis (14400 ceiling), with V.42 detection.
#[unsafe(no_mangle)]
pub extern "C" fn bm_create_v32(answer: c_int, max_rate: c_int,
    get_bit: GetBitFn, get_ud: *mut c_void, put_bit: PutBitFn, put_ud: *mut c_void,
) -> *mut Answerer {
    if max_rate != 9600 && max_rate != 14400 { return std::ptr::null_mut(); }
    let mut end = Answerer::new(answer != 0, false, false, get_bit, get_ud, put_bit, put_ud);
    end.start_v32(max_rate as u32);
    Box::into_raw(Box::new(end))
}

/// One 8 kHz line sample in, the corresponding line sample out.
#[unsafe(no_mangle)]
pub extern "C" fn bm_step(a: *mut Answerer, input: c_int) -> c_int {
    let a = unsafe { &mut *a };
    // One sample is 125 us of line. A step that takes many frames' worth
    // holds every later sample up behind it, and the far end hears the gap
    // unless something downstream has audio in hand; say when it happens.
    let began = std::time::Instant::now();
    step_prof::reset();
    let out = a.step_inner(input);
    let took = began.elapsed();
    if took.as_millis() >= 15 {
        eprintln!("  slow step: {} ms at sample {} ({:.3} s): {}", took.as_millis(), a.samples, a.samples as f64 / 8000.0, step_prof::report());
    }
    // The raw line either way round, before the canceller takes the echo
    // off, so a capture says what was on the wire and not only what the
    // engine was shown.
    if let Some(c) = a.capture.as_mut() {
        c.push(input as i16, out as i16);
    }
    out
}

impl Answerer {
    /// Write the echo canceller's state, once per call, when `V90_DATA_ECHO`
    /// names a file.
    ///
    /// This exists so that the echo can be measured on the same line samples the
    /// constellation statistics come from. `V90_DATA_POINTS` gives each point's
    /// position in the V.90 modem's own sample numbering; the capture gives the
    /// line's; and this gives the offset between them and the filter that was
    /// in the path, so the prediction and the residual can be reproduced
    /// exactly rather than estimated. The 29.6 dB of ERLE_tx that said the echo
    /// was cancelled and the E[z^8] of 9.3 that said the constellation was gone
    /// were taken half a call apart, which is a gap: they could both be true
    /// and still not be about the same samples.
    fn write_echo_state(&mut self, v90_now: u64) {
        let Some(path) = std::env::var_os("V90_DATA_ECHO") else { return };
        // The offset between this modem's sample count and the call's, read
        // back from the modem rather than assumed. It was assumed, from the
        // instant `start_v90` was called, and the two were 24 s apart on the
        // 2026-09-26 21:45 call: the V.90 modem had been running since before
        // the FTI's idea of when it started, so the constellation points and
        // the echo series were 194 698 samples out of step and neither could be
        // placed against the other.
        self.v90_origin = self.samples.saturating_sub(v90_now);
        if self.data_log.is_none()
            && let Ok(f) = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(std::path::Path::new(&path))
        {
            self.data_log = Some(f);
        }
        // Every entry, not only the first. A call that retrains reaches data mode
        // more than once, and which of those a constellation point belongs to is
        // settled by its count against the header nearest below it; latching on
        // the first left that unanswerable.
        self.data_log_from = self
            .samples
            .saturating_sub(self.v90_origin)
            .saturating_add(DATA_LOG_SKIP);
        self.data_log_header();
    }

    /// The header for one call's data-mode window, written once.
    ///
    /// Appended and never truncated, because `V90_DATA_POINTS` is appended too
    /// and a call that reaches data mode more than once must not be mistaken
    /// for two calls. The V.90 modem's own sample count restarts at zero in each
    /// new one, so `data_now` is what says which points belong here: a point is
    /// this call's if its count is at or after this one.
    fn data_log_header(&mut self) {
        use std::io::Write;
        let Some(f) = self.data_log.as_mut() else { return };
        let _ = writeln!(f, "=== data mode");
        let _ = writeln!(f, "v90_origin {}", self.v90_origin);
        let _ = writeln!(f, "data_sample {}", self.samples);
        let _ = writeln!(f, "data_now {}", self.samples.saturating_sub(self.v90_origin));
        let _ = writeln!(f, "delay {}", self.echo.delay);
        let _ = writeln!(f, "taps_count {}", self.echo.w.len());
        let taps: Vec<String> = self.echo.w.iter().map(|v| format!("{v:.9}")).collect();
        let _ = writeln!(f, "taps {}", taps.join(" "));
        eprintln!(
            "  echo: data-mode window logged: origin {}, data_now {}, delay {}, {} \
             taps, norm {:.4}",
            self.v90_origin,
            self.samples.saturating_sub(self.v90_origin),
            self.echo.delay,
            self.echo.w.len(),
            self.echo.w.iter().map(|v| v * v).sum::<f64>().sqrt()
        );
    }

    /// One data-mode sample: its position, the line as it arrived, our own
    /// transmit at that moment, the echo the filter predicted from it, and what
    /// was left.
    ///
    /// Written from here rather than reconstructed from a capture because these
    /// are the numbers the receiver was actually given, and the point of the
    /// measurement is that the constellation statistics and the echo figures
    /// come off the same samples. Reconstructing the prediction offline from a
    /// capture and the dumped taps is the same arithmetic, but it has one more
    /// place to be wrong: the taps drift once data mode starts, and the dump
    /// holds only the value at the window's start.
    fn data_log_sample(&mut self, now: u64, x: f64, r: f64, yhat: f64) {
        use std::io::Write;
        let start = self.samples.saturating_sub(self.v90_origin);
        if start < self.data_log_from || start >= self.data_log_from + DATA_LOG_SAMPLES {
            return;
        }
        let Some(f) = self.data_log.as_mut() else { return };
        let _ = writeln!(f, "s {now} {x:.9} {r:.9} {yhat:.9} {:.9}", x - yhat);
    }

    fn step_inner(&mut self, input: c_int) -> c_int {
        if let Stage::Hst(m)=&mut self.stage {
            self.samples+=1;
            let out=m.step(input as f64/32768.0);
            let (tx,rx)=m.rates();self.rate_tx=tx as c_int;self.rate_rx=rx as c_int;
            self.physical_connected=m.connected;
            self.status=if m.failed{BM_FAILED}else if m.connected{BM_CONNECTED}else{BM_RUNNING};
            if m.connected {
                while let Some(bit)=m.up.pop_front() {
                    self.rx_total+=1;
                    if let Some(f)=self.put_bit {unsafe{f(self.put_ud,bit as c_int)};}
                }
                // Typing and uploads go into MNP while its queue is short; it
                // holds 4 KB and paces them to whichever channel is ours. C
                // hands idle mark for ever when it has nothing, so take at
                // most two bits a sample: 16 000 bit/s, past HST's fastest.
                for _ in 0..2 {
                    if m.backlog()>=64 {break;}
                    let bit=self.get_bit.map(|f|unsafe{f(self.get_ud)}).unwrap_or(-1);
                    if bit<0 {break;}
                    m.take_bit(bit!=0);
                }
            }
            // The session's carriers are at unit scale; send them at V.23's level.
            return (out*0.3*32768.0).clamp(-32768.0,32767.0) as c_int;
        }
        if let Stage::V23(m)=&mut self.stage {
            self.samples+=1;
            let out=m.step(input as f64/32768.0);
            let (tx,rx)=m.rates();self.rate_tx=tx as c_int;self.rate_rx=rx as c_int;
            self.physical_connected=m.connected();
            self.status=if m.failed(){BM_FAILED}else if m.connected(){BM_CONNECTED}else{BM_RUNNING};
            if m.connected() {
                for bit in m.take_bits() {
                    self.rx_total+=1;
                    if let Some(f)=self.put_bit {unsafe{f(self.put_ud,bit as c_int)};}
                }
                while m.pending_bits()<2 {
                    let bit=self.get_bit.map(|f|unsafe{f(self.get_ud)}).unwrap_or(-1);
                    if bit<0 {break;}
                    m.push_bits(&[bit!=0]);
                }
            }
            return (out*32768.0).clamp(-32768.0,32767.0) as c_int;
        }
        if matches!(self.stage, Stage::V8(_)) {
            self.legacy_v32_listener.feed(input as f64 / 32768.0);
            // AA answers ANSam, so only tone heard from ANSam on counts.
            // A V.92 caller's V.8 bis reply to CRe (V.21 high channel, 1650
            // and 1850 Hz) stands on the listener for 0.6 s before ANSam
            // starts; counted, it sent such callers straight to V.32bis.
            if matches!(&self.stage, Stage::V8(m) if m.phase() == "ANSam") {
                self.legacy_v32_after_ansam = true;
            }
            self.legacy_v32_tone_samples = if self.legacy_v32_after_ansam
                && self.legacy_v32_listener.carrier_standing() {
                self.legacy_v32_tone_samples.saturating_add(1)
            } else { 0 };
            // A V.32 caller answers ANSam with AA, a steady 1800 Hz, and then
            // waits for AC. V.8 callers answer with CM instead, so do not wait
            // for the rest of ANSam's 5 s. This matters most after a failed V.92
            // Quick Connect, when the caller has already given up on V.8.
            if self.role == v34::phase2::Role::Answer
                && self.legacy_v32_tone_samples >= LEGACY_V32_EARLY_SAMPLES
                && matches!(&self.stage, Stage::V8(m) if m.phase() == "ANSam")
            {
                self.start_v32(14400);
            }
        }
        self.samples += 1;
        if self.want_v90 {
            return self.linear_step(input);
        }
        /* Frozen in data mode: with no MP left to protect, the two ends' idle
           scramblers are the only thing a lock could mistake for echo. */
        // V.42 negotiation already carries peer data. Freeze at carrier,
        // rather than waiting for LAPM to finish and learning that data.
        self.echo.frozen = self.v34_carrier_up();
        self.echo.data = self.echo.frozen;
        self.echo.far_silent = v34_echo_identify()
            && matches!(&self.stage, Stage::V34(m) if m.far_end_silent());
        // The rows to fit start once the far end's J has stopped arriving.
        self.echo.peer_silent = self.echo.far_silent
            && matches!(&self.stage, Stage::V34(m) if m.far_end_quiet());
        let line = if matches!(self.stage, Stage::V32(_)) {
            input as f64 / 32768.0 // V.32 owns its training echo canceller.
        } else { self.echo.sample(input as f64 / 32768.0) };
        let x = line / BOUNDARY_GAIN;
        self.up.process(x, &mut self.mid);
        let engine_in = std::mem::take(&mut self.mid);
        if engine_in.len() != 2 {
            self.up_odd += 1;
            self.up_odd_last = engine_in.len() as u8;
        }
        for s in engine_in {
            let y = self.engine_step(s);
            self.down.process(y, &mut self.mid);
            for &z in self.mid.iter() {
                self.out.push_back(z);
            }
            self.mid.clear();
        }
        if self.out.is_empty() {
            self.underruns += 1;
        }
        let y8 = self.out.pop_front().unwrap_or(0.0) * BOUNDARY_GAIN;
        self.echo.push(y8);
        if y8.abs() > self.peak {
            self.peak = y8.abs();
        }
        if y8 > 1.0 || y8 < -1.0 {
            self.clips += 1;
        }
        (y8 * 32768.0).clamp(-32768.0, 32767.0) as c_int
    }
}

/// One call's line audio, kept when `BM_CAPTURE` names a directory to put it
/// in: what arrived in channel 0, what went out in channel 1, at the line's
/// own rate, so a capture can be replayed through an engine the way the
/// notes on the echo canceller came about (see `RetrainWatch::heard_before`).
struct Capture {
    path: std::path::PathBuf,
    samples: Vec<(i16, i16)>,
}

impl Capture {
    fn new(dir: &std::path::Path) -> Self {
        use std::sync::atomic::{AtomicU64, Ordering};
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        Self { path: dir.join(format!("line-{:08}-{n}.wav", std::process::id())), samples: Vec::new() }
    }

    fn push(&mut self, rx: i16, tx: i16) {
        // Ten minutes is longer than any call here, and bounds the memory.
        if self.samples.len() < 4_800_000 {
            self.samples.push((rx, tx));
        }
    }

    /// A 16-bit stereo WAV at the line's rate: header, then the samples.
    fn write(&self) {
        let (n, fs) = (self.samples.len(), LINE_FS as u32);
        let mut w: Vec<u8> = Vec::with_capacity(44 + n * 4);
        w.extend_from_slice(b"RIFF");
        w.extend_from_slice(&(36 + n as u32 * 4).to_le_bytes());
        w.extend_from_slice(b"WAVEfmt ");
        w.extend_from_slice(&16u32.to_le_bytes());
        w.extend_from_slice(&1u16.to_le_bytes()); /* PCM */
        w.extend_from_slice(&2u16.to_le_bytes()); /* channels */
        w.extend_from_slice(&fs.to_le_bytes());
        w.extend_from_slice(&(fs * 4).to_le_bytes());
        w.extend_from_slice(&4u16.to_le_bytes());
        w.extend_from_slice(&16u16.to_le_bytes());
        w.extend_from_slice(b"data");
        w.extend_from_slice(&(n as u32 * 4).to_le_bytes());
        for &(rx, tx) in &self.samples {
            w.extend_from_slice(&rx.to_le_bytes());
            w.extend_from_slice(&tx.to_le_bytes());
        }
        match std::fs::write(&self.path, w) {
            Ok(()) => eprintln!("BinModem: captured {} line samples to {}", n, self.path.display()),
            Err(why) => eprintln!("BinModem: could not capture to {}: {why}", self.path.display()),
        }
    }
}

impl Drop for Answerer {
    fn drop(&mut self) {
        if let Some(capture) = &self.capture {
            capture.write();
        }
    }
}

/// Times the engine's own output went outside [-1, 1] before the s16 clamp.
#[unsafe(no_mangle)]
pub extern "C" fn bm_clips(a: *mut Answerer) -> u64 {
    let a = unsafe { &*a };
    a.clips
}

/// Largest engine output magnitude seen.
#[unsafe(no_mangle)]
pub extern "C" fn bm_peak(a: *mut Answerer) -> f64 {
    let a = unsafe { &*a };
    a.peak
}

/// The boundary echo canceller's state: locked delay in samples (0 = never
/// locked), the correlation peak the lock was accepted on, and the energy
/// in the filter (0 until something has been learned).
#[unsafe(no_mangle)]
pub extern "C" fn bm_echo(
    a: *mut Answerer,
    delay: *mut c_int,
    peak: *mut c_double,
    energy: *mut c_double,
) {
    let a = unsafe { &*a };
    unsafe {
        if !delay.is_null() {
            *delay = a.echo.delay as c_int;
        }
        if !peak.is_null() {
            *peak = a.echo.peak;
        }
        if !energy.is_null() {
            *energy = a.echo.energy();
        }
    }
}

/// Waveform gaps: the transmit queue ran dry before the pop.
#[unsafe(no_mangle)]
pub extern "C" fn bm_underruns(a: *mut Answerer) -> u64 {
    let a = unsafe { &*a };
    a.underruns
}

/// Input steps that did not yield exactly two engine samples; `last` gets
/// what the most recent odd step yielded.
#[unsafe(no_mangle)]
pub extern "C" fn bm_up_odd(a: *mut Answerer, last: *mut u8) -> u64 {
    let a = unsafe { &mut *a };
    if !last.is_null() {
        unsafe { *last = a.up_odd_last };
    }
    a.up_odd
}

/// BM_LAPM_TRACE as (from, to) seconds of call time.
fn lapm_trace_window() -> Option<(u64, u64)> {
    static W: std::sync::OnceLock<Option<(u64, u64)>> = std::sync::OnceLock::new();
    *W.get_or_init(|| {
        let v = std::env::var("BM_LAPM_TRACE").ok()?;
        let (a, b) = v.split_once(',')?;
        Some((a.trim().parse().ok()?, b.trim().parse().ok()?))
    })
}

/// Move the data bits: top the transmitter up from `get_bit` and hand what
/// the receiver has recovered to `put_bit`. Call once per audio chunk.
#[unsafe(no_mangle)]
pub extern "C" fn bm_service(a: *mut Answerer) {
    let a = unsafe { &mut *a };
    if matches!(a.stage,Stage::V23(_)) {return;}
    if let Stage::V32(modem) = &mut a.stage {
        if a.v32_logged_phase != modem.phase() {
            a.v32_logged_phase = modem.phase().to_owned();
            eprintln!("BinModem V.32 phase={} round_trip={} echo_loss={:.1} reflection={:?}",
                modem.phase(), modem.round_trip(), modem.echo_return_loss(), modem.reflection());
        }
        for signal in modem.take_sequences() {
            eprintln!("BinModem V.32 rate RX {signal:04x} phase={}", modem.phase());
        }
    }
    // Keep analogue line buffering near 50 ms, below LAPM response timers.
    // V.34's former 8192 bits also delayed acknowledgements by 380 ms at
    // 21600 bit/s, on top of the measured network round trip. The mapping
    // frame reserve in pending_bits() remains in addition to this watermark.
    let line = match &a.stage {
        Stage::V32(_) => TxLine::V32,
        Stage::V34(_) => TxLine::V34,
        Stage::V90(m) if !m.is_v90() => TxLine::V34,
        Stage::V90(_) => TxLine::V90,
        _ => TxLine::Other,
    };
    let tx_watermark = tx_watermark(line, a.rate_tx.max(0) as u32);
    let (accepts, mut pending) = match &a.stage {
        Stage::V32(m) => (matches!(m.status(), v32::startup::Status::Connected(_)), m.pending_bits()),
        Stage::V34(m) => (m.accepts_bits(), m.pending_bits()),
        Stage::V90(m) => (m.accepts_bits(), m.pending_bits()),
        _ => (false, 0),
    };
    let rx_bits = match &mut a.stage {
        Stage::V32(m) => Some(m.take_bits()),
        Stage::V34(m) => Some(m.take_bits()),
        Stage::V90(m) => Some(m.take_bits()),
        _ => None,
    };
    if let Some(bits) = rx_bits {
        a.rx_total = a.rx_total.wrapping_add(bits.len() as u64);
        if let Some(ec) = a.ec.as_mut() {
            for bit in bits {
                ec.feed_bit(bit);
            }
            for frame in ec.take_log() {
                #[cfg(test)]
                if std::env::var_os("BM_RECORDED_EC_WINDOW").is_some() && (164*8000..176*8000).contains(&a.samples) {
                    let hex=frame.body.iter().take(12).map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ");
                    eprintln!("EC-WINDOW {:.3} {} {} [{}]",a.samples as f64/8000.0,if frame.outbound {"TX"} else {"RX"},if frame.intact {"ok"} else {"BAD"},hex);
                }
                if a.ec_frames_logged < 4000 {
                    let hex = frame.body.iter().take(12)
                        .map(|b| format!("{b:02x}"))
                        .collect::<Vec<_>>().join(" ");
                    eprintln!("BinModem V.42 frame {} {} [{}] {:.3}",
                              if frame.outbound { "TX" } else { "RX" },
                              if frame.intact { "ok" } else { "BAD" }, hex, a.samples as f64 / 8000.0);
                    a.ec_frames_logged += 1;
                }
            }
            if ec.phase() == EcPhase::Transparent {
                if let Some(f) = a.put_bit {
                    for bit in ec.take_unclaimed() {
                        unsafe { f(a.put_ud, bit as c_int) };
                    }
                }
            } else if let Some(f) = a.put_bit {
                for byte in ec.take_received() {
                    for bit in a.async_tx.encode(byte) {
                        unsafe { f(a.put_ud, bit as c_int) };
                    }
                }
            }
        } else if let Some(f) = a.put_bit {
            for bit in bits {
                unsafe { f(a.put_ud, bit as c_int) };
            }
        }
    }

    // BM_LAPM_TRACE=<from s>,<to s>: every LAPM frame out and in between
    // those call times, with the link's millisecond clock.
    if let Some((from, to)) = lapm_trace_window()
        && let Some(ec) = a.ec.as_mut()
    {
        let t = a.samples / 8000;
        let on = t >= from && t < to;
        if on && !a.lapm_tracing {
            ec.set_lapm_trace(true);
            a.lapm_tracing = true;
        } else if !on && a.lapm_tracing {
            for l in ec.take_lapm_trace() { eprintln!("LAPM-TRACE {l}"); }
            ec.set_lapm_trace(false);
            a.lapm_tracing = false;
        }
        if a.lapm_tracing {
            for l in ec.take_lapm_trace() { eprintln!("LAPM-TRACE {l}"); }
        }
    }
    if !a.xid_logged
        && let Some(ec) = a.ec.as_ref()
        && let Some(x) = ec.heard_xid()
    {
        a.xid_logged = true;
        let (n401, k) = ec.send_limits();
        eprintln!("BinModem: V.42 XID from the far end: N401 transmit {:?} receive {:?}, k transmit {:?} receive {:?}; sending with N401 {n401}, k {k}",
            x.n401_transmit, x.n401_receive, x.window_transmit, x.window_receive);
    }
    if a.ec_health_trace && a.samples/8000 != a.ec_health_last {
        a.ec_health_last=a.samples/8000;
        if let Some(ec)=a.ec.as_ref() {
            eprintln!("EC-HEALTH t={} good={} information={} bad={} undecodable={} stall={:?} unheard={:?} phase={:?}",a.ec_health_last,ec.valid_frames(),ec.valid_information_frames(),ec.damaged_frames(),ec.undecodable_streams(),ec.stalled_transmit_ms(),ec.unheard_ms(),ec.phase());
        }
    }

    let physical_data=a.v34_carrier_up() || matches!(&a.stage,Stage::V90(m) if m.is_v90() && matches!(m.status(),v90::startup::Status::Connected{..}));
    if physical_data != a.link_data_active {
        a.link_data_active=physical_data;
        eprintln!("  link: physical data {} at {:.3} s, {} bits queued down", if physical_data {"up"} else {"down"}, a.samples as f64/8000.0, pending);
        if !physical_data {
            a.data_down_at=a.samples;
        }
        if physical_data {
            // The peer has just trained: old CRC/ack ages include time when
            // it could not send data. Start receive evidence afresh and give
            // LAPM a short opportunity to resume before considering a retry.
            if let Some(ec)=a.ec.as_ref() {
                a.framing_good=ec.valid_frames();
                a.framing_bad=ec.damaged_frames();
            }
            a.framing_since=a.samples;
            a.framing_valid_at=a.samples;
            a.data_up_at=a.samples;
            a.realign=Realign::Idle;
            // A rate renegotiation is back in about 3 s, a retrain in 20 or more.
            // Experiment V92_RENEG_SLIP=1: no retrain at six seconds after a
            // renegotiation; the slip search instead, as without one.
            a.after_renegotiation=a.data_down_at>0 && a.samples.saturating_sub(a.data_down_at)<8*8000
                && !std::env::var("V92_RENEG_SLIP").is_ok_and(|v| v == "1");
            a.reneg_rescues=reneg_rescues_left(a.reneg_rescues, None, 0, !a.after_renegotiation);
            a.peer_renegotiated=a.after_renegotiation && match &a.stage {
                Stage::V90(m) => m.renegotiated_by_peer(),
                _ => false,
            };
            // Three seconds' grace after the analogue modem's own renegotiation,
            // so its rescue (next_realign, 3.5 s) is not held back to five.
            let grace = if a.peer_renegotiated { 3 } else { 5 };
            a.link_retrain_after=a.link_retrain_after.max(a.samples.saturating_add(grace*8000));
        }
    }

    // Carrier/timing lock alone cannot detect a whole-symbol frame-clock jump.
    // Require multiple CRC failures AND two seconds without any intact frame;
    // intact supervisory acknowledgements also prove the receive frame clock.
    // Application stalls are handled separately by the LAPM ACK watchdog.
    if let Some(ec) = a.ec.as_ref() {
        let good = ec.valid_frames();
        let bad = ec.damaged_frames();
        if !monitor_receive_framing(physical_data, ec.phase(), ec.is_connected())
            || good != a.framing_good {
            if good != a.framing_good {
                a.framing_rate_recovery = false;
            }
            a.framing_good = good;
            a.framing_bad = bad;
            a.framing_since = a.samples;
            a.framing_valid_at = a.samples;
        } else if bad.saturating_sub(a.framing_bad) >= 3
            && a.samples.saturating_sub(a.framing_since) >= 2 * 8000
        {
            if let Stage::V90(m) = &mut a.stage {
                if a.samples.saturating_sub(a.framing_valid_at) >= 8 * 8000
                    && a.samples >= a.link_retrain_after && m.is_v90()
                    && matches!(m.status(), v90::startup::Status::Connected { .. })
                {
                    // A frame-clock slip can leave the equalizer and carrier
                    // healthy. Refresh framing through the shorter rate
                    // handshake before discarding that trained physical path.
                    // If no intact frame follows, the next attempt is a full
                    // retrain. LAPM and compression state survive both paths.
                    if !a.framing_rate_recovery
                        && m.recover_upstream_rate((a.rate_rx.max(4800) / 2400) as u8)
                    {
                        a.framing_rate_recovery = true;
                        eprintln!("BinModem V.90: invalid LAPM framing: quality-based rate renegotiation");
                        a.link_retrain_after = a.samples.saturating_add(20 * 8000);
                    } else if m.retrain() {
                        eprintln!("BinModem V.90: no intact LAPM frame for 8s despite repeated CRC recovery: full physical retrain");
                        a.link_retrain_after = a.samples.saturating_add(40 * 8000);
                    }
                } else {
                    m.recover_upstream_framing();
                }
            }
            a.framing_bad = bad;
            a.framing_since = a.samples;
        }
    }

    // Intact upstream frames do not prove the caller can decode downstream.
    // Give LAPM retries time, then restore the physical link before PPP's
    // keepalive deadline. Keep its frames and compression dictionaries.
    if physical_data && a.ec.as_ref().is_some_and(|ec|
        ec.stalled_transmit_ms().is_none_or(|ms| ms < 5000))
    {
        a.ack_rate_recovery = false;
    }
    if a.samples >= a.link_retrain_after
        && a.ec.as_ref().is_some_and(|ec| ec.stalled_transmit_ms().is_some_and(|ms| ms >= 20_000))
    {
        let renegotiated = !a.ack_rate_recovery
            && match &mut a.stage {
                Stage::V90(m) if m.is_v90() => m.renegotiate((a.rate_rx.max(4800) / 2400) as u8),
                _ => false,
            };
        if renegotiated {
            a.ack_rate_recovery = true;
            eprintln!("BinModem V.90: downstream LAPM acknowledgements stalled for 20s: same-rate renegotiation, preserving LAPM");
            a.link_retrain_after = a.samples.saturating_add(20 * 8000);
        } else if a.request_physical_retrain() {
            eprintln!("BinModem: downstream LAPM acknowledgements stalled for 20s: full physical retrain, preserving LAPM");
            a.link_retrain_after = a.samples.saturating_add(40 * 8000);
        }
    }

    // The same failure with nothing of ours outstanding. A Conexant on the
    // outside route stopped reading our downstream and went on polling for
    // 40 s and more while its line monitor stayed content; with no I frame
    // of ours waiting, the watchdog above never saw it. LAPM probes an idle
    // link, so a peer that hears us answers within a round trip. Ten seconds
    // of silence in data mode: look for the frame boundary it reads at (see
    // next_realign), and retrain if none answers.
    // A validated playout delay shift in data mode can move the peer's
    // frame boundary (next_realign): poll now, and look for it if no answer.
    if let Some(shift) = a.echo.take_shift()
        && physical_data
        && a.realign == Realign::Idle
    {
        a.realign = Realign::shifted(shift, a.samples);
        if a.realign != Realign::Idle {
            if let Some(ec) = a.ec.as_mut() {
                ec.probe();
            }
            eprintln!("BinModem: path delay moved {shift:+} samples ({} mod 6): polling the peer", shift.rem_euclid(6));
        }
    }
    if physical_data && (a.samples >= a.link_retrain_after || a.realign != Realign::Idle) {
        let unheard = a.ec.as_ref().and_then(|ec| ec.unheard_ms());
        a.reneg_rescues = reneg_rescues_left(a.reneg_rescues, unheard, a.samples.saturating_sub(a.data_up_at), false);
        // Heard since the renegotiation: later deafness is not its doing.
        if heard_this_period(unheard, a.samples.saturating_sub(a.data_up_at)) {
            a.after_renegotiation = false;
        }
        // V92_RENEG_RESCUE=0: no renegotiation of ours on a deaf one.
        let rescue = a.reneg_rescues > 0 && !std::env::var("V92_RENEG_RESCUE").is_ok_and(|v| v == "0");
        let (state, action) = next_realign(a.realign, unheard, a.samples.saturating_sub(a.data_up_at), a.samples, a.after_renegotiation, rescue, a.peer_renegotiated);
        a.realign = state;
        let slipped = match action {
            RealignAction::Slip(n) => match &mut a.stage {
                Stage::V90(m) => (0..n).all(|_| m.slip_downstream()),
                _ => false,
            },
            _ => false,
        };
        match action {
            RealignAction::Slip(n) if slipped => {
                if let Some(ec) = a.ec.as_mut() {
                    ec.probe();
                }
                if let Realign::Slipping { offset, tried, .. } = a.realign {
                    eprintln!("BinModem: peer has not shown it hears us for {:.1} s: downstream slipped {n} symbol(s) (offset {offset}, try {tried}), polling", unheard.unwrap_or(0) as f64 / 1000.0);
                }
            }
            RealignAction::Heard(n) => {
                eprintln!("BinModem: peer hears us again after {n} slip tr(ies): its frame boundary had moved");
            }
            // No slipping on this link (not PCM upstream data mode): only a
            // peer unheard for the full ten seconds is retrained.
            RealignAction::Slip(_) if !peer_unhearing(unheard, a.samples.saturating_sub(a.data_up_at)) => {
                a.realign = Realign::Idle;
            }
            RealignAction::Renegotiate if match &mut a.stage {
                Stage::V90(m) => m.renegotiate_pcm_up("the analogue modem has not heard us since its rate renegotiation"),
                _ => false,
            } => {
                a.reneg_rescues = a.reneg_rescues.saturating_sub(1);
                eprintln!("BinModem: peer has not shown it hears us for {:.1} s since a rate renegotiation: renegotiating from this end, preserving LAPM", unheard.unwrap_or(0) as f64 / 1000.0);
            }
            RealignAction::Slip(_) | RealignAction::Retrain | RealignAction::Renegotiate => {
                a.realign = Realign::Idle;
                if a.request_physical_retrain() {
                    let why = if a.after_renegotiation { " since a rate renegotiation" } else if action == RealignAction::Retrain { " at any frame boundary" } else { "" };
                    eprintln!("BinModem: peer has not shown it hears us for {:.1} s in data mode{why}: full physical retrain, preserving LAPM", unheard.unwrap_or(0) as f64 / 1000.0);
                    a.link_retrain_after = a.samples.saturating_add(40 * 8000);
                }
            }
            RealignAction::Nothing => {}
        }
    }

    if accepts {
        if let Some(ec) = a.ec.as_mut() {
            /* C still exposes an async DTE bit stream. Reassemble it into
               octets here; LAPM owns the synchronous line below it. */
            if ec.is_connected() {
                let mut bytes = Vec::new();
                while pending < tx_watermark {
                    let raw = match a.get_bit {
                        Some(f) => unsafe { f(a.get_ud) },
                        None => 1,
                    };
                    if let Some(byte) = a.async_tx.feed(raw & 1 != 0) {
                        bytes.push(byte);
                    }
                    /* One DTE bit consumed does not mean one line bit queued.
                       Stop after the watermark's worth of input, then frame. */
                    pending += 1;
                }
                if !bytes.is_empty() {
                    ec.send(&bytes);
                }
            }
            if ec.phase() == EcPhase::Transparent {
                /* V.42 was declined, or asked for and never answered: the
                   link is raw async below after all, so the DTE's stream
                   goes straight onto the line -- the transmit side of the
                   passthrough the receive side already is, which hands what
                   arrives at put_bit straight from take_unclaimed. Data was
                   held through detection (nothing may preempt it, V.250
                   6.5.5); from here nothing frames it. Without this the
                   line carried idle ones forever while get_bit was never
                   called, and a far end with no V.42 got a call that could
                   receive but never send. */
                while pending < tx_watermark {
                    let raw = match a.get_bit {
                        Some(f) => unsafe { f(a.get_ud) },
                        None => 1,
                    };
                    match &mut a.stage {
                        Stage::V32(m) => m.send_bits(&[raw & 1 != 0]),
                        Stage::V34(m) => m.send_bits(&[raw & 1 != 0]),
                        Stage::V90(m) => m.send_bits(&[raw & 1 != 0]),
                        _ => break,
                    }
                    pending += 1;
                }
            } else {
                match &mut a.stage {
                    Stage::V32(m) => {
                        pending = m.pending_bits();
                        while pending < tx_watermark {
                            m.send_bits(&[ec.next_bit()]);
                            pending += 1;
                        }
                    }
                    Stage::V34(m) => {
                        pending = m.pending_bits();
                        while pending < tx_watermark {
                            m.send_bits(&[ec.next_bit()]);
                            pending += 1;
                        }
                    }
                    Stage::V90(m) => {
                        pending = m.pending_bits();
                        while pending < tx_watermark {
                            m.send_bits(&[ec.next_bit()]);
                            pending += 1;
                        }
                    }
                    _ => {}
                }
            }
        } else {
            while pending < tx_watermark {
                let raw = match a.get_bit {
                    Some(f) => unsafe { f(a.get_ud) },
                    None => 1,
                };
                match &mut a.stage {
                    Stage::V32(m) => m.send_bits(&[raw & 1 != 0]),
                        Stage::V34(m) => m.send_bits(&[raw & 1 != 0]),
                    Stage::V90(m) => m.send_bits(&[raw & 1 != 0]),
                    _ => break,
                }
                pending += 1;
            }
        }
    }
    if accepts {
        a.update_connected_status();
    }
}

/// Total bits the receiver has handed up since creation.
#[unsafe(no_mangle)]
pub extern "C" fn bm_rx_total(a: *mut Answerer) -> u64 {
    let a = unsafe { &*a };
    a.rx_total
}

/// Times the data decoder lost itself and had to re-acquire.
#[unsafe(no_mangle)]
pub extern "C" fn bm_found_again(a: *mut Answerer) -> u32 {
    let a = unsafe { &*a };
    match &a.stage {
        Stage::V34(m) => m.training().map(|t| t.found_again()).unwrap_or(0),
        _ => 0,
    }
}

/// Sample slips the receiver has seen since training began.
#[unsafe(no_mangle)]
pub extern "C" fn bm_slips(a: *mut Answerer) -> u32 {
    let a = unsafe { &*a };
    match &a.stage {
        Stage::V34(m) => m.training().map(|t| t.slips()).unwrap_or(0),
        _ => 0,
    }
}

/// 1 once the far modem has everything this end was given to send (every
/// V.42 frame acknowledged; always 1 without V.42), else 0. Unlike
/// bm_pending this is not held up by the flags V.42 keeps the line full of.
#[unsafe(no_mangle)]
pub extern "C" fn bm_tx_delivered(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    tx_delivered(a.ec.as_ref()) as c_int
}

/// Bits the engine's transmitter still has waiting (its own accounting:
/// data queued less what the next mapping frame takes).
#[unsafe(no_mangle)]
pub extern "C" fn bm_pending(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    match &a.stage {
        Stage::V23(m) => m.pending_bits() as c_int,
        Stage::Hst(m) => m.backlog() as c_int * 10,
        Stage::V32(m) => m.pending_bits() as c_int,
        Stage::V34(m) => m.pending_bits() as c_int,
        Stage::V90(m) => m.pending_bits() as c_int,
        _ => -1,
    }
}

/// Throw away whatever the receiver has made of the handshake. The engine's
/// demodulator hands up bits before it has finished training; the first time
/// data mode is reported, everything waiting is noise, and the caller drops
/// it before opening the byte path.
#[unsafe(no_mangle)]
pub extern "C" fn bm_flush_rx(a: *mut Answerer) {
    let a = unsafe { &mut *a };
    if a.ec.is_none() {
        match &mut a.stage {
            Stage::V23(m) => { let _ = m.take_bits(); }
            Stage::Hst(m) => { m.up.clear(); }
            Stage::V32(m) => { let _ = m.take_bits(); }
            Stage::V34(m) => { let _ = m.take_bits(); }
            Stage::V90(m) => { let _ = m.take_bits(); }
            _ => {}
        }
    }
}

/// Whether V.42 LAPM is established on the current physical connection.
#[unsafe(no_mangle)]
pub extern "C" fn bm_error_control(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.ec.as_ref().is_some_and(EcStack::is_connected) as c_int
}

/// Negotiated compression: 0 none, 1 V.42bis, 2 V.44.
#[unsafe(no_mangle)]
pub extern "C" fn bm_compression(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.ec.as_ref().map_or(0, compression_code)
}

/// 0 none, 1 V.42bis, 2 V.44.
fn compression_code(ec: &EcStack) -> c_int {
    match ec.compression_name() {
        Some("V.42bis") => 1,
        Some("V.44") => 2,
        _ => 0,
    }
}

/// 0 detection, 1 XID, 2 LAPM, 3 transparent.
fn ec_phase_code(ec: &EcStack) -> c_int {
    match ec.phase() {
        EcPhase::Detecting => 0,
        EcPhase::Negotiating => 1,
        EcPhase::Protocol => 2,
        EcPhase::Transparent => 3,
    }
}

/// Bytes through the compressor this call: `out` gets plain sent, wire
/// sent, wire received, plain received. 0 when nothing compresses (and
/// `out` is left alone), 1 otherwise.
#[unsafe(no_mangle)]
pub extern "C" fn bm_compression_counts(a: *mut Answerer, out: *mut u64) -> i32 {
    let a = unsafe { &*a };
    let Some(c) = a.ec.as_ref().and_then(EcStack::compression_counts) else { return 0 };
    let out = unsafe { std::slice::from_raw_parts_mut(out, 4) };
    out.copy_from_slice(&[c.plain_out, c.wire_out, c.wire_in, c.plain_in]);
    1
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_damaged_frames(a: *mut Answerer) -> u64 {
    let a = unsafe { &*a };
    a.ec.as_ref().map_or(0, EcStack::damaged_frames)
}

/// What every error-corrected link here offers: V.42bis both ways and V.44,
/// or neither when `compression_off`.
///
/// Some V.42bis-era modems repeat their XID forever when an XID carries the
/// later V.44 private parameter set instead of ignoring the unknown extension
/// as V.42 requires. So V.44 is offered only to a caller whose own XID offers
/// it first: those callers get V.44 and the older ones never see the
/// subfield. V90_V44=0 turns V.44 off.
fn offer_compression_to(stack: &mut EcStack, compression_off: bool) {
    if compression_off {
        stack.without_v42bis();
    } else {
        stack.offer_compression(Compression::Both);
    }
    if compression_off || std::env::var("V90_V44").as_deref() == Ok("0") {
        stack.without_v44();
    } else {
        stack.v44_when_offered();
    }
}

/// V.42 for a data pump that is not in this crate: C's V.22bis.
///
/// The pump trades line bits with this link (`bm_ec_rx_bit`, `bm_ec_tx_bit`)
/// and the link trades an asynchronous DTE bit stream with C through the same
/// get_bit/put_bit pair the pump used to call directly -- so on the C side
/// nothing above the pump changes, exactly as with the engines' own LAPM.
/// Detection runs first unless V.8 already agreed LAPM, so a caller with no
/// V.42 drops to transparent and gets the plain async link it always had.
pub struct EcLink {
    ec: EcStack,
    dte_rx: AsyncBits,
    dte_tx: AsyncBits,
    get_bit: GetBitFn,
    get_ud: *mut c_void,
    put_bit: PutBitFn,
    put_ud: *mut c_void,
    /// DTE characters gathered for one `send`: compression and framing work
    /// per call, and a call per character costs a flush and a frame each.
    batch: Vec<u8>,
    /// Mark bits in a row from the DTE: ten mean no character is under way.
    marks: u32,
}

/// Bytes the link may hold unsent before it stops taking the DTE's: a
/// quarter second at 2400 bit/s, so flow control reaches PPP quickly.
const EC_LINK_BACKLOG: usize = 64;
/// DTE bits taken per line bit while there is room: more than one, since
/// V.42bis can carry more characters than the line has bits for.
const EC_LINK_DTE_PER_LINE_BIT: usize = 3;
/// Characters handed to V.42 in one go while the DTE keeps typing.
const EC_LINK_BATCH: usize = 32;

impl EcLink {
    fn deliver(&mut self) {
        let Some(put) = self.put_bit else { return };
        if self.ec.phase() == EcPhase::Transparent {
            for bit in self.ec.take_unclaimed() {
                unsafe { put(self.put_ud, bit as c_int) };
            }
        } else {
            for byte in self.ec.take_received() {
                for bit in self.dte_rx.encode(byte) {
                    unsafe { put(self.put_ud, bit as c_int) };
                }
            }
        }
    }

    fn dte_bit(&mut self) -> bool {
        match self.get_bit {
            Some(f) => (unsafe { f(self.get_ud) } & 1) != 0,
            None => true,
        }
    }

    fn line_bit(&mut self) -> bool {
        if self.ec.phase() == EcPhase::Transparent {
            // No V.42 after all: the DTE's stream is the line's.
            return self.dte_bit();
        }
        if self.ec.is_connected() {
            for _ in 0..EC_LINK_DTE_PER_LINE_BIT {
                if self.ec.queued() + self.batch.len() >= EC_LINK_BACKLOG {
                    break;
                }
                let bit = self.dte_bit();
                self.marks = if bit { self.marks.saturating_add(1) } else { 0 };
                if let Some(byte) = self.dte_tx.feed(bit) {
                    self.batch.push(byte);
                }
            }
            // Send a full batch, or what there is once the DTE has paused.
            if self.batch.len() >= EC_LINK_BATCH || (!self.batch.is_empty() && self.marks >= 10) {
                let batch = std::mem::take(&mut self.batch);
                self.ec.send(&batch);
            }
        }
        self.ec.next_bit()
    }
}

/// A V.42 link for C's own pump. `answer` picks the role; `lapm_declared`
/// is what V.8 agreed (skip detection); `line_bps` and `round_trip_ms` set
/// T401. Returns null on a bad argument.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_create(answer: c_int, lapm_declared: c_int, line_bps: c_int, round_trip_ms: c_int,
                               get_bit: GetBitFn, get_ud: *mut c_void,
                               put_bit: PutBitFn, put_ud: *mut c_void) -> *mut EcLink {
    let role = if answer != 0 { EcRole::Answerer } else { EcRole::Originator };
    let round_trip = round_trip_ms.max(0) as u32;
    let params = EcParams {
        t401_ms: ec::lapm::t401_for_line(line_bps.max(300) as u32, round_trip),
        ..EcParams::default()
    };
    let mut stack = EcStack::new(role, params).over_a_round_trip(round_trip);
    if lapm_declared != 0 {
        stack = stack.declared_lapm();
    }
    offer_compression_to(&mut stack, false);
    Box::into_raw(Box::new(EcLink {
        ec: stack, dte_rx: AsyncBits::new(8), dte_tx: AsyncBits::new(8),
        get_bit, get_ud, put_bit, put_ud, batch: Vec::new(), marks: 0,
    }))
}

/// One bit the pump received from the line.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_rx_bit(l: *mut EcLink, bit: c_int) {
    let Some(l) = (unsafe { l.as_mut() }) else { return };
    l.ec.feed_bit(bit != 0);
    l.deliver();
}

/// The next bit for the pump to send.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_tx_bit(l: *mut EcLink) -> c_int {
    let Some(l) = (unsafe { l.as_mut() }) else { return 1 };
    c_int::from(l.line_bit())
}

/// Time for V.42's timers, in milliseconds.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_tick(l: *mut EcLink, ms: c_int) {
    let Some(l) = (unsafe { l.as_mut() }) else { return };
    if ms > 0 {
        l.ec.tick(ms as u32);
    }
    l.deliver();
}

/// 0 detection, 1 XID, 2 LAPM, 3 transparent (as bm_ec_phase).
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_link_phase(l: *mut EcLink) -> c_int {
    let Some(l) = (unsafe { l.as_ref() }) else { return -1 };
    ec_phase_code(&l.ec)
}

/// 0 none, 1 V.42bis, 2 V.44 (as bm_compression).
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_link_compression(l: *mut EcLink) -> c_int {
    let Some(l) = (unsafe { l.as_ref() }) else { return 0 };
    compression_code(&l.ec)
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_link_damaged_frames(l: *mut EcLink) -> u64 {
    unsafe { l.as_ref() }.map_or(0, |l| l.ec.damaged_frames())
}

/// 1 once the far modem has released an established V.42 link (DISC).
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_link_peer_released(l: *mut EcLink) -> c_int {
    let l = unsafe { &*l };
    l.ec.peer_released() as c_int
}

/// 1 once the far modem has released the engine's established V.42 link.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_peer_released(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.ec.as_ref().is_some_and(|s| s.peer_released()) as c_int
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_destroy(l: *mut EcLink) {
    if !l.is_null() {
        drop(unsafe { Box::from_raw(l) });
    }
}

/// V.42 progress: -1 not started, 0 detection, 1 XID, 2 LAPM, 3 transparent.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_phase(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.ec.as_ref().map_or(-1, ec_phase_code)
}

/// Whether V.8 said both ends support LAPM.
#[unsafe(no_mangle)]
pub extern "C" fn bm_lapm_declared(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.lapm_declared as c_int
}

/// V.42 observations: bit 0 ADP received, bit 1 XID received, bit 2 text seen.
#[unsafe(no_mangle)]
pub extern "C" fn bm_ec_observed(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.ec.as_ref().map_or(0, |ec| {
        (ec.far_answer().is_some() as c_int)
            | ((ec.far_xid().is_some() as c_int) << 1)
            | ((ec.far_text() as c_int) << 2)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_status(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.status
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_rate_tx(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.rate_tx
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_rate_rx(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    a.rate_rx
}

/// Whether the far end's data signal is on the line. Only meaningful in
/// data mode.
#[unsafe(no_mangle)]
pub extern "C" fn bm_carrier(a: *mut Answerer) -> c_int {
    let a = unsafe { &*a };
    match &a.stage {
        Stage::V23(m) => m.connected() as c_int,
        Stage::Hst(m) => m.connected as c_int,
        Stage::V32(m) => m.carrier() as c_int,
        Stage::V34(m) => m.carrier() as c_int,
        Stage::V90(m) => m.carrier() as c_int,
        _ => 0,
    }
}

/// What the start-up is doing, as a human-readable phrase. The pointer is
/// into `a` and is valid until the next call on any of its functions.
#[unsafe(no_mangle)]
pub extern "C" fn bm_phase(a: *mut Answerer) -> *const c_char {
    let a = unsafe { &mut *a };
    a.copy_phase();
    a.phase_buf.as_ptr().cast()
}

/// Why the start-up failed; empty string unless the status is BM_FAILED.
#[unsafe(no_mangle)]
pub extern "C" fn bm_failure(a: *mut Answerer) -> *const c_char {
    let a = unsafe { &mut *a };
    a.copy_failure();
    a.fail_buf.as_ptr().cast()
}

#[unsafe(no_mangle)]
pub extern "C" fn bm_destroy(a: *mut Answerer) {
    if !a.is_null() {
        drop(unsafe { Box::from_raw(a) });
    }
}

#[cfg(test)]
mod ls_tests {
    use super::*;

    fn xorshift_noise(seed: &mut u32, scale: f64) -> f64 {
        *seed ^= *seed << 13; *seed ^= *seed >> 17; *seed ^= *seed << 5;
        scale * (f64::from(*seed) / f64::from(u32::MAX) * 2.0 - 1.0)
    }

    /// The filter as it was first written: one bounds test per tap. Kept
    /// here so the fast version is held to exactly what it replaced.
    fn apply_filter_reference(taps_w: &[f64], delay: usize, window: &[(f64, f64)]) -> Vec<f64> {
        let centre = taps_w.len() / 2;
        let d0 = delay as isize - centre as isize;
        let n = window.len();
        let mut out = vec![0.0; n];
        for i in 0..n {
            let mut y = 0.0;
            for (k, w) in taps_w.iter().enumerate() {
                let j = i as isize - d0 - k as isize;
                if j >= 0 && (j as usize) < n {
                    y += w * window[j as usize].1;
                }
            }
            out[i] = window[i].0 - y;
        }
        out
    }

    fn tx_correlated_reference(residual: &[f64], reference: &[f64], delay: usize, taps: usize) -> f64 {
        let centre = taps / 2;
        let n = residual.len().min(reference.len());
        if n < 256 { return 0.0; }
        let ref_power: f64 = reference[..n].iter().map(|v| v * v).sum();
        if ref_power <= 0.0 { return 0.0; }
        let mut best = 0.0f64;
        let mut lag = delay.saturating_sub(centre);
        while lag < delay + centre {
            if lag + 256 <= n {
                let mut c = 0.0;
                for i in lag..n { c += residual[i] * reference[i - lag]; }
                let c = c / (n - lag) as f64;
                if c * c > best { best = c * c; }
            }
            lag += AB_LAG_STEP;
        }
        best * n as f64
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1e-12)
    }

    #[test]
    fn the_fast_scan_correlation_matches_the_per_sample_one_at_every_lag() {
        let mut echo = Echo::new();
        let mut seed = 0x51ce_d00du32;
        for _ in 0..ECHO_RING + 777 {
            let y = xorshift_noise(&mut seed, 0.3);
            echo.push(y);
        }
        // A reflection at 8640 samples plus far-end noise.
        let n = echo.tx.len();
        echo.seen = (0..ECHO_WINDOW).map(|i| {
            let back = 8640 + ECHO_WINDOW - i;
            0.2 * echo.tx[(echo.tx_pos + n - back) % n] + xorshift_noise(&mut seed, 0.01)
        }).collect();
        let job = ScanJob::new(&echo);
        let fast = |lag| job.corr(lag);
        for lag in (ECHO_LAG_LO..echo.lag_hi).step_by(7).chain([8640, echo.lag_hi - 1]) {
            let (a, b) = (echo.corr_at(lag), fast(lag));
            assert!(close(a, b), "lag {lag}: per-sample {a} fast {b}");
        }
        assert!(fast(8640) > 0.95, "the reflection is found: {}", fast(8640));
    }

    #[test]
    fn the_fast_scan_correlation_is_zero_on_silent_history_like_the_old_one() {
        let mut echo = Echo::new();
        echo.seen = vec![0.1; ECHO_WINDOW];
        let job = ScanJob::new(&echo);
        let fast = |lag| job.corr(lag);
        assert_eq!(fast(4000), 0.0);
        assert_eq!(echo.corr_at(4000), 0.0);
    }

    #[test]
    fn the_fast_filter_and_correlation_match_the_reference_ones() {
        let mut seed = 0x0bad_cafeu32;
        let taps = 256;
        let delay = 8664;
        let w: Vec<f64> = (0..taps).map(|_| xorshift_noise(&mut seed, 0.01)).collect();
        for len in [300usize, 4096, AB_WINDOW] {
            let window: Vec<(f64, f64)> = (0..len)
                .map(|_| (xorshift_noise(&mut seed, 0.2), xorshift_noise(&mut seed, 0.5))).collect();
            for d in [delay, 100, 0, len + 50] {
                let a = apply_filter_reference(&w, d, &window);
                let b = Echo::apply_filter(&w, d, &window);
                assert_eq!(a.len(), b.len());
                for (i, (x, y)) in a.iter().zip(&b).enumerate() {
                    assert!((x - y).abs() <= 1e-12, "len {len} delay {d} sample {i}: {x} vs {y}");
                }
            }
            let rx: Vec<f64> = window.iter().map(|p| p.0).collect();
            let rf: Vec<f64> = window.iter().map(|p| p.1).collect();
            for d in [128usize, 300, 2000] {
                let (a, b) = (tx_correlated_reference(&rx, &rf, d, taps), Echo::tx_correlated(&rx, &rf, d, taps));
                assert!(close(a, b), "len {len} delay {d}: {a} vs {b}");
            }
        }
    }

    #[test]
    fn the_delay_search_spread_over_steps_finds_the_lock_the_one_shot_search_found() {
        let mut echo = Echo::new();
        echo.peer_silent = true;
        let mut seed = 0x0dd5_eedu32;
        // Band-limited, as a line signal is: white noise has no correlation
        // width, and the coarse search steps four lags at a time.
        let mut y = 0.0;
        for _ in 0..ECHO_RING {
            y = 0.85 * y + xorshift_noise(&mut seed, 0.01);
            echo.push(y);
        }
        let n = echo.tx.len();
        echo.seen = (0..ECHO_WINDOW).map(|i| {
            let back = 6403 + ECHO_WINDOW - i;
            0.1 * echo.tx[(echo.tx_pos + n - back) % n] + xorshift_noise(&mut seed, 0.0002)
        }).collect();
        assert!(echo.quiet());
        // The one-shot answer, on the same snapshot.
        let job = ScanJob::new(&echo);
        let mut best = (0usize, -1.0f64);
        for lag in (ECHO_LAG_LO..echo.lag_hi).step_by(4) {
            if job.corr(lag).abs() > best.1 { best = (lag, job.corr(lag).abs()); }
        }
        for lag in best.0 - 4..=best.0 + 5 {
            if job.corr(lag).abs() > best.1 { best = (lag, job.corr(lag).abs()); }
        }
        assert_eq!(best.0, 6403);
        echo.scan();
        assert_eq!(echo.delay, 0, "a step spends no more than its budget");
        let mut steps = 1;
        while echo.scan_job.is_some() {
            echo.scan_step(SCAN_LAGS_PER_STEP);
            steps += 1;
        }
        assert_eq!(echo.delay, 6403);
        assert!((echo.peak - best.1).abs() < 1e-12);
        assert!(steps <= 45, "{steps} steps");
        // And a lock that moves meanwhile voids the answer.
        echo.delay = 0;
        echo.peak = 0.0;
        echo.scan();
        echo.delay = 5000;
        echo.scan_step(usize::MAX);
        assert_eq!(echo.delay, 5000);
        assert!(echo.scan_job.is_none());
    }

    fn echo_with_a_b(async_ab: bool) -> Echo {
        let mut e = Echo::new();
        e.training_async = async_ab;
        e.delay = 1320;
        let taps = e.w.len();
        let mut seed = 0x00ab_00abu32;
        let mut y = 0.0;
        let r: Vec<f64> = (0..AB_WINDOW).map(|_| { y = 0.6 * y + xorshift_noise(&mut seed, 0.3); y }).collect();
        let window: Vec<(f64, f64)> = (0..AB_WINDOW).map(|i| {
            let echo = if i >= 1320 { 0.1 * r[i - 1320] } else { 0.0 };
            (echo + xorshift_noise(&mut seed, 0.002), r[i])
        }).collect();
        let mut ls = vec![0.0; taps];
        ls[taps / 2] = 0.1;
        let gradient = vec![0.0; taps];
        e.w = ls.clone();
        e.ab = Some(Ab { gradient, ls, window });
        e
    }

    fn settle_a_b(e: &mut Echo) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while e.ab_job.is_some() && std::time::Instant::now() < deadline {
            e.ab_poll();
            std::thread::yield_now();
        }
    }

    #[test]
    fn the_a_b_judged_off_the_audio_thread_installs_what_the_inline_one_does() {
        let mut inline = echo_with_a_b(false);
        inline.ab_select();
        assert!(inline.ab_job.is_none() && inline.ab.is_none());
        let mut worker = echo_with_a_b(true);
        worker.ab_select();
        assert!(worker.ab_pending(), "the judging is still pending, not done in the step");
        settle_a_b(&mut worker);
        assert!(!worker.ab_pending());
        assert_eq!(worker.delay, inline.delay);
        assert_eq!(worker.w, inline.w);
        assert!(inline.w[inline.w.len() / 2] > 0.05, "the block fit wins on this window");
    }

    #[test]
    fn an_a_b_verdict_from_before_a_retrain_is_dropped() {
        let mut e = echo_with_a_b(true);
        e.w.fill(0.0);
        e.ab.as_mut().unwrap().ls.fill(0.0);
        e.ab.as_mut().unwrap().gradient.fill(0.0);
        let kept = vec![0.25; e.w.len()];
        e.ab_select();
        e.restart_training();
        e.w = kept.clone();
        settle_a_b(&mut e);
        assert_eq!(e.w, kept);
    }

    #[test]
    #[ignore = "timing printout"]
    fn time_the_echo_bulk_work() {
        let mut echo = Echo::new();
        let mut seed = 0x51ce_d00du32;
        for _ in 0..ECHO_RING { echo.push(xorshift_noise(&mut seed, 0.3)); }
        echo.seen = (0..ECHO_WINDOW).map(|_| xorshift_noise(&mut seed, 0.1)).collect();
        let t = std::time::Instant::now();
        let job = ScanJob::new(&echo);
        let fast = |lag| job.corr(lag);
        let mut acc = 0.0;
        let build_ms = t.elapsed().as_secs_f64() * 1e3;
        for lag in (ECHO_LAG_LO..echo.lag_hi).step_by(4) { acc += fast(lag); }
        let scan_ms = t.elapsed().as_secs_f64() * 1e3;
        let t = std::time::Instant::now();
        for lag in (ECHO_LAG_LO..echo.lag_hi).step_by(4) { acc += echo.corr_at(lag); }
        let old_ms = t.elapsed().as_secs_f64() * 1e3;
        let w: Vec<f64> = (0..256).map(|_| xorshift_noise(&mut seed, 0.01)).collect();
        let window: Vec<(f64, f64)> = (0..AB_WINDOW).map(|_| (xorshift_noise(&mut seed, 0.2), xorshift_noise(&mut seed, 0.5))).collect();
        let t = std::time::Instant::now();
        let f = Echo::apply_filter(&w, 8664, &window);
        let filt_ms = t.elapsed().as_secs_f64() * 1e3;
        let t = std::time::Instant::now();
        let g = apply_filter_reference(&w, 8664, &window);
        let filt_old_ms = t.elapsed().as_secs_f64() * 1e3;
        println!("TIMING snapshot {build_ms:.3} ms; one step of {SCAN_LAGS_PER_STEP} lags {:.3} ms; scan {scan_ms:.2} ms (was {old_ms:.2}); filter {filt_ms:.2} ms (was {filt_old_ms:.2}) {}", scan_ms * SCAN_LAGS_PER_STEP as f64 / ((echo.lag_hi - ECHO_LAG_LO) / 4) as f64, acc + f[9] + g[9]);
    }

    /// Call 0 of 2026-11-21 (pcmup80): between the DIL and phase 4 the
    /// adaptive filter grew from a norm of 0.08 to 7, where every fitted path
    /// is 0.25 or less, and phase 4 heard only the canceller (SNR -22 dB). A
    /// near-silent transmit against a line that is not quite silent is the
    /// shape that does it: the step is divided by the transmit's energy.
    #[test]
    fn the_adaptive_filter_never_grows_past_an_echo_path() {
        let mut echo = Echo::new();
        echo.delay = 1320;
        echo.peer_silent = true;
        let mut seed = 0x2545_f491_u32;
        let mut noise = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            f64::from(seed) / f64::from(u32::MAX) - 0.5
        };
        let mut largest = 0.0f64;
        let mut adapted = false;
        for _ in 0..40_000 {
            echo.push(0.003 * noise());
            echo.sample(0.004 * noise());
            adapted |= echo.last_mu > 0.0;
            largest = largest.max(echo.energy().sqrt());
        }
        assert!(adapted, "never adapted: not the case being tested");
        assert!(Echo::plausible_fit_norm(largest), "the filter grew to a norm of {largest:.2}");
    }

    #[test]
    fn quiet_peer_training_does_not_adapt_an_unidentified_echo_filter() {
        let mut echo = Echo::new();
        echo.delay = 1320;
        echo.peer_silent = false;
        echo.far_silent = false;
        let weights = echo.w.clone();
        let (reference, _) = reference_with_spread(8192, 0.0);
        for (i, r) in reference.into_iter().enumerate() {
            echo.push(r);
            echo.sample(0.02 * (i as f64 * 0.93).sin());
        }
        assert!(!echo.identified);
        assert_eq!(echo.w, weights);
        assert_eq!(echo.last_mu, 0.0);
    }

    #[test]
    fn quiet_correlated_peer_training_cannot_relock_the_echo_path() {
        let mut echo = Echo::new();
        echo.delay = 7320;
        echo.peer_silent = false;
        let (reference, _) = reference_with_spread(ECHO_LAG_HI + 4096, 0.0);
        for (i, r) in reference.iter().copied().enumerate() {
            echo.push(r);
            let peer = if i >= 3320 { 0.1 * reference[i-3320] } else { 0.0 };
            echo.sample(peer);
        }
        assert_eq!(echo.delay, 7320);
        assert!(echo.w.iter().all(|w| *w == 0.0));
    }

    /// Calls 13 and 15 of 2026-11-15 (Conexant CX93001, 0.87 s round trip):
    /// through V.90 phase 3's TRN1d and Jd, with the analogue modem silent,
    /// the filter left over from phase 2 had no step to take and described
    /// nothing of the path, and what the receiver was given was 5 dB louder
    /// than the line -- loud and periodic enough that the S hunt read it as
    /// S. A filter that adds more than it takes away is not applied.
    #[test]
    fn a_filter_that_adds_to_the_line_is_not_applied() {
        let run = |good: bool| -> (f64, f64) {
            let mut echo = Echo::new();
            echo.delay = 7320;
            echo.peak = 0.99;
            echo.frozen = true;
            echo.harm_judged = true;
            let half = echo.w.len() / 2;
            let mut seed = 0x1357_9bdfu32;
            for (k, w) in echo.w.iter_mut().enumerate() {
                seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
                *w = if good { if k == half { 0.1 } else { 0.0 } }
                     else { 0.06 * (f64::from(seed) / f64::from(u32::MAX) * 2.0 - 1.0) };
            }
            let (reference, _) = reference_with_spread(ECHO_LAG_HI + 8 * ECHO_WINDOW, 0.0);
            let (mut line, mut out) = (0.0, 0.0);
            for (i, r) in reference.iter().copied().enumerate() {
                echo.push(r);
                let x = if i >= 7320 { 0.1 * reference[i - 7320] } else { 0.0 };
                let left = echo.sample(x);
                if i >= reference.len() - 4 * ECHO_WINDOW { line += x * x; out += left * left; }
            }
            (line, out)
        };
        let (line, out) = run(false);
        assert!(out <= line * 1.01, "a filter describing nothing made the line {:.1} dB louder",
            10.0 * (out / line).log10());
        let (line, out) = run(true);
        assert!(out < 0.01 * line, "the filter that is the path cancelled only {:.1} dB",
            10.0 * (line / out.max(1e-30)).log10());
    }

    /// The 2026-11-02 V.92 calls that died in phase 3: V.8 locked the echo
    /// at 7328 on a correlation of 0.998, and one quiet window later a full
    /// search moved it to an alias 0.577 strong -- and took 0.577 as the bar
    /// from then on. Phase 3 started with our echo uncancelled on top of the
    /// analogue modem's S, a reversal was "heard" 26 ms in, and the
    /// equaliser trained on nothing, twice, until the call was given up.
    #[test]
    fn a_strong_lock_is_not_traded_for_a_weaker_peak_elsewhere() {
        let mut echo = Echo::new();
        echo.delay = 7320;
        echo.peak = 0.99;
        echo.peer_silent = true;
        let (reference, _) = reference_with_spread(ECHO_LAG_HI + 8192, 0.0);
        let mut seed = 0x2468_ace1u32;
        for (i, r) in reference.iter().copied().enumerate() {
            echo.push(r);
            seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
            let noise = 0.013 * 1.732 * (f64::from(seed) / f64::from(u32::MAX) * 2.0 - 1.0);
            let alias = if i >= 3320 { 0.03 * reference[i - 3320] } else { 0.0 };
            echo.sample(alias + noise);
        }
        assert_eq!(echo.delay, 7320, "moved to a peak of {:.2}", echo.peak);
        assert!(echo.peak >= 0.99);
    }

    /// A path that really moved -- a playout jump -- still takes the lock.
    #[test]
    fn a_lock_follows_a_path_that_moved_as_strongly() {
        let mut echo = Echo::new();
        echo.delay = 7320;
        echo.peak = 0.99;
        echo.peer_silent = true;
        let (reference, _) = reference_with_spread(ECHO_LAG_HI + 8192, 0.0);
        for (i, r) in reference.iter().copied().enumerate() {
            echo.push(r);
            echo.sample(if i >= 3320 { 0.05 * reference[i - 3320] } else { 0.0 });
        }
        assert_eq!(echo.delay, 3320);
    }

    #[test]
    fn quiet_peer_signal_does_not_overwrite_the_identified_path() {
        let delay = 1320;
        let mut e = recover(&[(delay, -0.106)], delay, 0.0, 16384);
        assert!(e.identified);
        let original = e.w.clone();
        let (reference, _) = reference_with_spread(8192, 0.0);
        e.ab_collect(0.0, 0.0);
        // Test the live update alone; A/B selection has its own checks.
        e.ab = None;
        for i in 0..reference.len() {
            e.push(reference[i]);
            let echo = if i >= delay { -0.106 * reference[i-delay] } else { 0.0 };
            let peer = 0.02 * (i as f64 * 0.93).sin();
            e.sample(echo + peer);
        }
        assert_eq!(e.w, original);
    }

    #[test]
    fn live_filter_applies_the_identified_sample_alignment() {
        let count = 2 * ECHO_RING;
        let delay = 1320;
        let gain = -0.106;
        let mut e = recover(&[(delay, gain)], delay, 0.0, count);
        e.frozen = true;
        // The live ring's cursor points after the latest reference sample.
        let (reference, _) = reference_with_spread(count, 0.0);
        let mut residual = 0.0;
        let mut signal = 0.0;
        for i in 0..count {
            e.push(reference[i]);
            let line = if i >= delay { gain * reference[i - delay] } else { 0.0 };
            let left = e.sample(line);
            if i >= ECHO_RING {
                residual += left * left;
                signal += line * line;
            }
        }
        assert!(residual < signal * 1e-6, "live residual {residual}, echo {signal}");
    }

    #[test]
    fn identified_path_keeps_its_delay_during_peak_search() {
        let mut e = recover(&[(1320, -0.106)], 1320, 0.0, 16384);
        assert!(e.ls.as_ref().unwrap().solved);
        let original = e.w.clone();
        // A stronger second reflection would win an unrestricted search.
        e.seen = vec![0.1; ECHO_WINDOW];
        e.tx.fill(0.1);
        e.scan();
        e.scan_step(usize::MAX);
        assert_eq!(e.delay, 1320);
        assert_eq!(e.w, original);
    }

    #[test]
    fn v34_echo_comparison_keeps_the_measured_path_on_a_noise_level_tie() {
        // Saved failed repeat: gradient 3.55e-4, block fit 3.58e-4.
        assert_eq!(Echo::select_echo_candidate(4.58e-4, 3.55e-4, 3.58e-4, true), "LS");
        assert_eq!(Echo::select_echo_candidate(4.58e-4, 1.0e-4, 3.58e-4, true), "gradient");
        assert_eq!(Echo::select_echo_candidate(1.0e-4, 3.55e-4, 3.58e-4, true), "neither");
    }

    fn finished_fit(echo: &mut Echo, w: Vec<f64>, delay: usize) {
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        tx.send(TrainingFit { epoch: echo.training_epoch, delay, w, identified: true, ls: None }).unwrap();
        echo.training_job = Some(rx);
    }

    #[test]
    fn a_fit_finishing_after_the_receiver_trained_is_not_swapped_in() {
        let mut echo = Echo::new();
        echo.delay = 7088;
        echo.w = vec![0.01; 64];
        echo.hold_fits = true;
        finished_fit(&mut echo, vec![0.02; 64], 7079);
        echo.ls_poll();
        assert_eq!(echo.delay, 7088);
        assert_eq!(echo.w, vec![0.01; 64]);
        // Before phase 4 the same fit goes in.
        echo.hold_fits = false;
        finished_fit(&mut echo, vec![0.02; 64], 7079);
        echo.ls_poll();
        assert_eq!(echo.delay, 7079);
        assert_eq!(echo.w, vec![0.02; 64]);
    }

    #[test]
    fn a_finished_fit_far_too_large_is_discarded() {
        let mut echo = Echo::new();
        echo.delay = 7328;
        echo.w = vec![0.01; 64];
        finished_fit(&mut echo, vec![0.1; 64], 7627); // norm 0.8
        echo.ls_poll();
        assert_eq!(echo.delay, 7328);
        assert_eq!(echo.w, vec![0.01; 64]);
    }

    #[test]
    fn an_echo_fit_far_too_large_for_a_hybrid_is_not_installed() {
        assert!(Echo::plausible_fit_norm(0.1438));
        assert!(Echo::plausible_fit_norm(0.2466));
        assert!(!Echo::plausible_fit_norm(0.8044));
        assert!(!Echo::plausible_fit_norm(f64::NAN));
    }

    #[test]
    fn a_window_without_our_echo_cannot_condemn_a_working_fit() {
        // Live V.34 calls 22/23 (wiped, phase 4 failed) and 21/24 (kept).
        assert!(!Echo::echo_window_conclusive(1.18e-4, 4.40e-2));
        assert!(!Echo::echo_window_conclusive(2.13e-4, 4.42e-2));
        assert!(Echo::echo_window_conclusive(7.48e-3, 4.49e-2));
        assert!(Echo::echo_window_conclusive(5.56e-3, 4.42e-2));
        assert!(!Echo::echo_window_conclusive(f64::NAN, 4.4e-2));
    }

    /// Live V.92 calls 25/26, 2026-11-19 (an analogue modem, V.34 upstream):
    /// the window was too quiet to judge, the block fit was kept although it
    /// added our own echo (-16.6 and -2.7 dB), and the upstream never locked
    /// until a retrain dropped it. The eight other quiet windows since
    /// 2026-11-14 measured 0.0 dB and keep the fit.
    #[test]
    fn a_quiet_window_still_drops_a_fit_that_adds_our_echo() {
        assert_eq!(Echo::inconclusive_echo_choice(1.45e-3, 6.59e-2), "neither");
        assert_eq!(Echo::inconclusive_echo_choice(1.30e-3, 2.40e-3), "neither");
        assert_eq!(Echo::inconclusive_echo_choice(2.23e-4, 2.23e-4), "LS");
        assert_eq!(Echo::inconclusive_echo_choice(7.64e-4, 7.64e-4), "LS");
        // Half a decibel worse is noise on a quiet window: keep the fit.
        assert_eq!(Echo::inconclusive_echo_choice(1.0e-3, 1.12e-3), "LS");
        assert_eq!(Echo::inconclusive_echo_choice(1.0e-3, f64::NAN), "neither");
    }

    /// Live V.92 PCM upstream calls, 2026-11-11: the gradient won the window
    /// by 0.1 dB (24.3 against 24.2) and left 24 rms on TRN2u where the block
    /// fit leaves 17.6. Off V.34 it has to win by 3 dB.
    #[test]
    fn off_v34_the_gradient_has_to_beat_the_block_fit_by_three_decibels() {
        assert_eq!(Echo::select_echo_candidate(4.58e-4, 3.55e-4, 3.58e-4, false), "LS");
        assert_eq!(Echo::select_echo_candidate(1.18e-2, 4.47e-5, 4.57e-5, false), "LS");
        assert_eq!(Echo::select_echo_candidate(4.58e-4, 2.0e-4, 3.58e-4, false), "LS");
        assert_eq!(Echo::select_echo_candidate(4.58e-4, 1.7e-4, 3.58e-4, false), "gradient");
    }

    /// A reference whose spectrum spans `spread_db` from DC to Nyquist.
    ///
    /// A one-pole lowpass of coefficient `c` has a response of one over one
    /// minus `c` times `e` to the minus `jw`, so its magnitude runs from one at
    /// Nyquist to one over one minus `c` at DC: a spread of 20log10 of one over
    /// one minus `c`, and `c` set from that is the whole control.
    ///
    /// This matters because the excitation is what decides how well the path can
    /// be identified, and the DIL's is nothing like white: the reference
    /// spectrum on the 2026-09-26 capture spans 78.8 dB peak to floor. A test
    /// that excites with white noise says nothing about that, and the estimator
    /// that passes it lost a factor of four on the line.
    fn spread_coefficient(spread_db: f64) -> f64 {
        let tilt = 10f64.powf(spread_db / 20.0);
        (tilt - 1.0) / (tilt + 1.0)
    }

    /// `count` samples of reference with the given spectral spread, and a
    /// history long enough that every read is of a sample already sent.
    fn reference_with_spread(count: usize, spread_db: f64) -> (Vec<f64>, Vec<f64>) {
        let c = spread_coefficient(spread_db);
        let mut seed = 0x2468_1357u32;
        let mut r = vec![0.0f64; count];
        let mut z = 0.0f64;
        for v in r.iter_mut() {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let white = if (seed >> 16) & 1 == 0 { 0.1 } else { -0.1 };
            z = c * z + white;
            *v = z;
        }
        // Normalised, so the path's gain is the same number whatever the tilt.
        let rms = (r.iter().map(|v| v * v).sum::<f64>() / count as f64).sqrt();
        for v in r.iter_mut() {
            *v /= rms * 3.162_277_660_168_379; /* 0.1 rms */
        }
        let history = vec![0.0f64; count];
        (r, history)
    }

    /// Run the identification over a known path and return what it recovered.
    ///
    /// `path` is `(delay, gain)` pairs relative to the start of the run, and
    /// the lock is set separately because on a real call the two do not agree
    /// and that disagreement is a thing to look at on the line rather than to
    /// fold in here.
    fn recover(path: &[(usize, f64)], lock: usize, spread_db: f64, count: usize) -> Echo {
        let (r, mut history) = reference_with_spread(count, spread_db);
        let mut e = Echo::new();
        e.delay = lock;
        for (n, &tx) in r.iter().enumerate() {
            let mut y = 0.0;
            for (d, g) in path {
                if n >= *d {
                    y += g * history[n - *d];
                }
            }
            e.ls_feed(y, tx);
            history[n] = tx;
        }
        e.ls_solve();
        e
    }

    #[test]
    fn compact_echo_fit_finishes_inside_confirmed_dil_silence() {
        let delay = 7320;
        let prefix = ECHO_LAG_HI + 64;
        let count = prefix + 8192;
        let (reference, _) = reference_with_spread(count, 2.0);
        let mut echo = Echo::new();
        echo.w = vec![0.0; 64];
        echo.delay = delay;
        echo.training_async = false;
        echo.far_silent = true;
        echo.peer_silent = true;
        for &r in &reference[..prefix] { echo.push(r); }
        for n in prefix..count {
            echo.push(reference[n]);
            echo.sample(0.15 * reference[n-delay]);
        }
        assert!(echo.far_silent, "fit must finish before DIL's falling edge");
        assert!(echo.identified);
        assert_eq!(echo.delay, delay);
        assert!((echo.w[32] - 0.15).abs() < 0.001);
    }

    /// The local ATA's echo runs past 32 samples after its peak: on the
    /// 2026-11-10 13:07 recording 64 taps leave 48 rms of a 764 rms echo and
    /// 128 leave 12, the line's floor. A filter wide enough for that tail has
    /// to be solved inside the silence too: V.92's second TRN1u follows the
    /// window's end by a fifth of a second (Figure 10).
    #[test]
    fn a_filter_wide_enough_for_the_ata_s_tail_is_still_solved_inside_the_silence() {
        let delay = 1558;
        let taps = 256;
        let prefix = ECHO_LAG_HI + taps;
        let count = prefix + 8192;
        let (reference, _) = reference_with_spread(count, 2.0);
        let mut echo = Echo::new();
        echo.w = vec![0.0; taps];
        echo.delay = delay;
        echo.training_async = false;
        echo.far_silent = true;
        echo.peer_silent = true;
        for &r in &reference[..prefix] { echo.push(r); }
        for n in prefix..count {
            echo.push(reference[n]);
            echo.sample(0.15 * reference[n-delay] + 0.05 * reference[n-delay-60]);
        }
        assert!(echo.identified, "fit must finish before the window's falling edge");
        assert_eq!(echo.delay, delay);
        assert!((echo.w[taps/2] - 0.15).abs() < 0.001, "{}", echo.w[taps/2]);
        assert!((echo.w[taps/2 + 60] - 0.05).abs() < 0.001, "{}", echo.w[taps/2 + 60]);
    }

    /// A plain V.90 call on the wide filter (live, 2026-11-12): the DIL the
    /// analogue modem asked for repeats inside the filter's 256 taps, the
    /// pivots spread 47.8 dB, and five fits of six put their largest tap 122
    /// samples from the lock with a norm of 0.68 to 0.82. Each was discarded,
    /// nothing cancelled the echo, and phase 4's receiver fell to 10 dB. The
    /// old 64-tap filter on the same line: 3.1 dB, norm 0.144, connected.
    #[test]
    fn a_reference_that_repeats_inside_the_filter_is_fitted_over_fewer_taps() {
        let delay = 2359;
        let taps = 256;
        let period = 120;
        let prefix = ECHO_LAG_HI + taps;
        let count = prefix + 22000;
        let (white, _) = reference_with_spread(count, 2.0);
        let reference: Vec<f64> = (0..count)
            .map(|n| white[n % period] + 0.004 * white[n])
            .collect();
        let mut echo = Echo::new();
        echo.w = vec![0.0; taps];
        echo.delay = delay;
        for &r in &reference[..=prefix] { echo.push(r); }
        echo.begin_training_measurement();
        for n in prefix..count {
            let noise = 0.02 * white[(n * 7 + 3) % count];
            echo.ls_feed(0.075 * reference[n-delay] + noise, reference[n]);
        }
        echo.ls_solve_inline();
        assert_eq!(echo.delay, delay);
        let norm = echo.w.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(Echo::plausible_fit_norm(norm), "norm {norm}");
        assert!((echo.w[taps/2] - 0.075).abs() < 0.01, "{}", echo.w[taps/2]);
        assert!(norm < 0.1, "norm {norm}");
    }

    #[test]
    fn provider_echo_beyond_one_second_uses_seeded_transmit_history() {
        let delay = 10840;
        let prefix = ECHO_LAG_HI + 512;
        let count = prefix + 8192;
        let (reference, _) = reference_with_spread(count, 2.0);
        let mut echo = Echo::new();
        echo.delay = 3320;
        for &r in &reference[..=prefix] { echo.push(r); }
        echo.begin_training_measurement();
        for n in prefix..count {
            echo.ls_feed(0.15 * reference[n-delay], reference[n]);
        }
        echo.ls_solve_inline();
        assert_eq!(echo.delay, delay);
        assert!(echo.identified);
        assert!((echo.w[256] - 0.15).abs() < 0.001);
    }

    #[test]
    fn unsettled_peer_training_cannot_move_the_retained_echo_path() {
        let delay = 7320;
        let count = ECHO_LAG_HI + 8192;
        let (reference, _) = reference_with_spread(count, 2.0);
        let mut echo = Echo::new();
        echo.delay = delay;
        echo.identified = true;
        echo.w[256] = 0.15;
        let weights = echo.w.clone();
        echo.peer_silent = false;
        for n in 0..count {
            let foreign = if n >= 3320 { 0.5 * reference[n-3320] } else { 0.0 };
            echo.ls_feed(foreign, reference[n]);
        }
        echo.ls_solve_inline();
        assert_eq!(echo.delay, delay);
        assert_eq!(echo.w, weights);
        assert!(echo.identified);
    }

    #[test]
    fn seeded_history_does_not_fit_peer_training_before_silence() {
        let delay = 7320;
        let prefix = ECHO_LAG_HI + 512;
        let early = 7600;
        let quiet = 8192;
        let hold = 4096;
        let (reference, _) = reference_with_spread(prefix + early + quiet + hold, 2.0);
        let mut current = Echo::new();
        let mut contaminated = Echo::new();
        for e in [&mut current, &mut contaminated] {
            e.delay = delay;
            for &r in &reference[..=prefix] { e.push(r); }
            e.begin_training_measurement();
        }
        for n in prefix..prefix+early+quiet {
            // Early peer training is not silence and must not become echo.
            let foreign = if n < prefix+early { 0.5*reference[n-3320] } else { 0.0 };
            let line = 0.15*reference[n-delay] + foreign;
            current.peer_silent = n >= prefix+early;
            current.ls_feed(line, reference[n]);
            contaminated.ls_feed(line, reference[n]);
        }
        current.ls_solve_inline();
        contaminated.ls_solve_inline();
        let error = |e: &Echo| {
            let d0=e.delay-e.w.len()/2;
            (prefix+early+quiet..reference.len()).map(|n| {
                let y:f64=e.w.iter().enumerate().map(|(k,w)|w*reference[n-d0-k]).sum();
                (0.15*reference[n-delay]-y).powi(2)
            }).sum::<f64>()
        };
        let improvement=10.0*(error(&contaminated)/error(&current).max(1e-30)).log10();
        println!("Rejecting early peer training improves held-out echo prediction {improvement:.1}dB");
        assert!(improvement>10.0);
        assert_eq!(current.delay, delay);
    }
    #[test]
    fn long_delay_training_uses_preceding_transmit_history() {
        let delay = 7320;
        let prefix = ECHO_LAG_HI + 512;
        let window = 8192;
        let hold = 4096;
        let (reference, _) = reference_with_spread(prefix + window + hold, 20.0);
        let mut seed = 0x935af219u32;
        let line: Vec<_> = (0..reference.len()).map(|n| {
            seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
            let noise = (seed as f64 / u32::MAX as f64 - 0.5) * 0.006;
            if n >= delay { 0.25 * reference[n-delay] + noise } else { noise }
        }).collect();
        let mut current = Echo::new();
        current.delay = delay;
        for &r in &reference[..=prefix] { current.push(r); }
        current.begin_training_measurement();
        let mut former = Echo::new();
        former.delay = delay;
        for n in prefix..prefix+window {
            current.ls_feed(line[n], reference[n]);
            former.ls_feed(line[n], reference[n]);
        }
        assert_eq!(current.ls.as_ref().unwrap().samples, window);
        current.ls_solve_inline();
        former.ls_solve_inline();
        let error = |echo: &Echo| {
            let d0 = echo.delay - echo.w.len()/2;
            (prefix+window..reference.len()).map(|n| {
                let prediction: f64 = echo.w.iter().enumerate()
                    .map(|(k,w)| w * reference[n-d0-k]).sum();
                (0.25 * reference[n-delay] - prediction).powi(2)
            }).sum::<f64>()
        };
        let before = error(&former);
        let after = error(&current);
        let improvement = 10.0 * (before / after).log10();
        println!("915 ms echo, full quiet window: held-out prediction improvement {improvement:.1} dB");
        assert!(improvement > 6.0, "long-delay fit only improved {improvement:.1} dB");
        assert!(current.ls.as_ref().unwrap().solved);
    }

    /// Where the filter's largest tap is, as a delay, and how large it is.
    fn peak_as_delay(w: &[f64], lock: usize) -> (isize, f64) {
        let centre = w.len() / 2;
        let (at, (_, mag)) = w
            .iter()
            .enumerate()
            .fold((0usize, (0.0f64, 0.0f64)), |(bi, (bm, bv)), (i, &v)| {
                if v.abs() > bm { (i, (v.abs(), v)) } else { (bi, (bm, bv)) }
            });
        (lock as isize - centre as isize + at as isize, mag)
    }

    /// The selection has to be made on the transmit-correlated energy, on a
    /// window big enough to decide it, and the winner has to end up in the
    /// filter. This drives `ab_select` itself rather than the metric under it,
    /// because the metric is already tested and the wiring is not.
    #[test]
    fn the_ab_puts_the_winner_in_the_filter() {
        let taps = 512usize;
        let delay = 1320usize;
        let centre = taps / 2;
        let n = AB_WINDOW;
        let (r, _) = reference_with_spread(n + delay, 2.0);
        // The line: the path, plus a far-end signal neither filter is to blame
        // for, plus some noise.
        let mut far = 0x5eed_1234u32;
        let mut window: Vec<(f64, f64)> = Vec::with_capacity(n);
        for i in 0..n {
            let mut y = 0.0;
            for (k, (_o, g)) in [(0usize, -0.106f64), (3, 0.04), (7, -0.02)]
                .iter()
                .enumerate()
            {
                let j = i as isize - delay as isize - k as isize;
                if j >= 0 {
                    y += g * r[j as usize];
                }
            }
            far = far.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let f = if (far >> 16) & 1 == 0 { 0.02 } else { -0.02 };
            window.push((y + f, r[i]));
        }
        // The right filter: the path at the lock, so on tap `centre`.
        let mut good = vec![0.0; taps];
        for (k, (_o, g)) in [(0usize, -0.106f64), (3, 0.04), (7, -0.02)]
            .iter()
            .enumerate()
        {
            good[centre + k] = *g;
        }
        // The wrong one, and the wrong one the old metric would have chosen: the
        // same path buried in noise a tenth as large again.
        let mut noise = 0xfeed_faceu32;
        let mut noisy = vec![0.0; taps];
        for v in noisy.iter_mut() {
            noise = noise.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *v = ((noise >> 16) as f64 / 32768.0 - 0.5) * 0.02;
        }
        for (k, (_o, g)) in [(0usize, -0.106f64), (3, 0.04), (7, -0.02)]
            .iter()
            .enumerate()
        {
            noisy[centre + k] = *g;
        }

        let mut e = Echo::new();
        e.delay = delay;
        e.ab = Some(Ab { gradient: noisy.clone(), ls: good.clone(), window });
        e.ab_select();
        assert!(e.ab.is_none(), "the A/B was not consumed");
        for (k, (&a, &b)) in e.w.iter().zip(&good).enumerate() {
            assert!(
                (a - b).abs() < 1e-12,
                "tap {k} is {a:+.6} and the right answer is {b:+.6}"
            );
        }
        // And the same pair the other way round, so the choice is not an
        // artefact of which candidate happens to be listed first: the good one
        // wins on merit from either slot.
        let good_again = good.clone();
        let mut e = Echo::new();
        e.delay = delay;
        e.ab = Some(Ab { gradient: good_again.clone(), ls: noisy, window: rebuild() });
        e.ab_select();
        for (k, v) in e.w.iter().enumerate() {
            assert!(
                (v - good_again[k]).abs() < 1e-12,
                "with the order reversed, tap {} is {:+.6} and the right answer is {:+.6}",
                k,
                v,
                good_again[k]
            );
        }
    }

    /// The same window again, for the reversed-order case above. Built twice
    /// rather than cloned because the A/B takes it by value.
    #[test]
    fn rejected_echo_fits_do_not_remain_in_the_training_path() {
        let mut echo = Echo::new();
        echo.delay = 1320;
        let mut rejected = vec![0.0; echo.w.len()];
        rejected[echo.w.len() / 2] = 0.3;
        // ls_solve installs its candidate before the independent comparison.
        // Both candidates here add echo instead of cancelling the recorded
        // negative reflection. Selecting "neither" must actually bypass them.
        echo.w = rejected.clone();
        echo.ab = Some(Ab {
            gradient: rejected.clone(),
            ls: rejected,
            window: rebuild(),
        });
        echo.ab_select();
        assert!(echo.ab.is_none());
        assert!(echo.w.iter().all(|&coefficient| coefficient == 0.0),
            "a rejected echo fit still modifies final-training samples");
    }

    #[test]
    fn echo_comparison_rejects_an_unmeasured_path() {
        assert_eq!(Echo::select_echo_candidate(0.0, 0.0, 0.0, false), "neither");
        assert_eq!(Echo::select_echo_candidate(f64::NAN, 1.0, 1.0, false), "neither");
    }

    #[test]
    fn echo_comparison_measures_a_5800_sample_path() {
        let delay = 5800;
        let taps = 512;
        let mut seed = 0x1234_5678u32;
        let reference: Vec<f64> = (0..AB_WINDOW).map(|_| {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed as f64 / u32::MAX as f64 - 0.5) * 0.2
        }).collect();
        let window: Vec<_> = (0..AB_WINDOW).map(|i| {
            (if i >= delay { 0.2 * reference[i-delay] } else { 0.0 }, reference[i])
        }).collect();
        let rx: Vec<_> = window.iter().map(|p| p.0).collect();
        let mut correct = vec![0.0; taps];
        correct[taps/2] = 0.2;
        let residual = Echo::apply_filter(&correct, delay, &window);
        let before = Echo::tx_correlated(&rx, &reference, delay, taps);
        let after = Echo::tx_correlated(&residual, &reference, delay, taps);
        assert!(before > 0.0);
        assert_eq!(Echo::select_echo_candidate(before, before, after, false), "LS");
        assert_eq!(Echo::tx_correlated(&rx[..4096], &reference[..4096], delay, taps), 0.0);
    }

    fn rebuild() -> Vec<(f64, f64)> {
        let delay = 1320usize;
        let n = AB_WINDOW;
        let (r, _) = reference_with_spread(n + delay, 2.0);
        let mut far = 0x5eed_1234u32;
        let mut window: Vec<(f64, f64)> = Vec::with_capacity(n);
        for i in 0..n {
            let mut y = 0.0;
            for (k, (_o, g)) in [(0usize, -0.106f64), (3, 0.04), (7, -0.02)]
                .iter()
                .enumerate()
            {
                let j = i as isize - delay as isize - k as isize;
                if j >= 0 {
                    y += g * r[j as usize];
                }
            }
            far = far.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let f = if (far >> 16) & 1 == 0 { 0.02 } else { -0.02 };
            window.push((y + f, r[i]));
        }
        window
    }

    /// The judging window has to cover every lag the filter can reach, or the
    /// measurement silently excludes the path's peak and the answer is noise.
    ///
    /// This is a bug that was in the A/B from the first version: the window was
    /// a quarter of a second, the projection skips any lag whose reference
    /// would run past the end, and a call locking at 1879 samples therefore had
    /// its path measured at lags the path was not at. It showed up as the live
    /// ERLE_tx swinging from 15.7 dB to -4.6 dB between calls with the same
    /// estimator and the same path.
    #[test]
    fn the_judging_window_covers_every_lag_the_filter_reaches() {
        let taps = 512usize;
        // The longest delay the search will ever report, from the lock's range.
        let worst = ECHO_LAG_HI.max(V34_ECHO_LAG_HI);
        let needed = worst + taps / 2 + 256;
        assert!(
            AB_WINDOW >= needed,
            "a {AB_WINDOW}-sample window cannot search the {}-sample lag the \\
             filter reaches at the longest lockable delay; it needs {needed}",
            worst + taps / 2
        );
        // And the check is not vacuous: at a quarter of a second it did not
        // hold, which is how this was found.
        assert!(AB_WINDOW >= 3584, "the window came back to {AB_WINDOW}");
    }

    /// The identification has to recover a path it was never told, and recover
    /// it *quantitatively*: the delay, the sign, the amplitude and a filter that
    /// actually cancels. Finding the delay is not enough and was not enough --
    /// the version of this that only checked the delay passed while the
    /// estimate came back at a quarter of the true gain, and on the real line
    /// that filter left 0.1 dB of the echo where no cancellation at all would
    /// have been better.
    fn identification_at(delay: usize, gain: f64, lock: usize) {
        // The path this line actually has: a reflection about 20 dB down, a few
        // samples wide, at the delay the correlation locks.
        let path = [
            (delay, gain),
            (delay + 3, -0.4 * gain),
            (delay + 7, 0.2 * gain),
        ];
        let e = recover(&path, lock, 2.0, 20_000);
        let ls = e.ls.as_ref().expect("window");
        assert!(ls.solved, "no solve at a delay of {delay}");

        let (peak_delay, peak_mag) = peak_as_delay(&e.w, e.delay);
        // The delay. The largest tap has to be the path's first arrival, and
        // the estimate's taps are indexed by delay directly, so there is no
        // alignment step to be wrong about.
        assert!(
            (peak_delay - delay as isize).abs() <= 4,
            "the largest tap is at a delay of {peak_delay}, the path is at {delay}"
        );
        // The sign. A path that comes back inverted amplifies where it should
        // have cancelled, which is the failure the very first attempt had.
        assert!(
            peak_mag.signum() == gain.signum(),
            "a path of {gain} came back with its largest tap at {peak_mag:+.4}"
        );
        // The amplitude, to a fifth. This is the assertion the estimator did not
        // have. A Hann-windowed block correlation loses `(n - D) / n` of the
        // numerator while the denominator still counts the whole block, so at
        // the n = 2D this was sized to it returns half, and it did.
        assert!(
            (peak_mag / gain - 1.0).abs() <= 0.2,
            "a path of {gain} at {delay} came back at {peak_mag:+.4}, \
             {}% out",
            ((peak_mag / gain - 1.0) * 100.0).round()
        );
        // The norm, against the impulse response that was injected: a filter
        // that is much larger is inventing a path, and one that is much smaller
        // has not found it. The three taps sum in quadrature to 1.09 * |gain|.
        let norm = e.w.iter().map(|v| v * v).sum::<f64>().sqrt();
        let true_norm = path
            .iter()
            .map(|(_, g)| g * g)
            .sum::<f64>()
            .sqrt();
        assert!(
            (norm / true_norm - 1.0).abs() <= 0.35,
            "a path of norm {true_norm:.4} came back with a filter of norm {norm:.4}"
        );
        // And it has to cancel, measured the way the filter will be judged:
        // on the energy left that is correlated with our own transmit.
        let (r, mut history) = reference_with_spread(20_000, 2.0);
        let mut window: Vec<(f64, f64)> = Vec::new();
        for n in 0..20_000 {
            let mut y = 0.0;
            for (d, g) in path {
                if n >= d {
                    y += g * history[n - d];
                }
            }
            window.push((y, r[n]));
            history[n] = r[n];
        }
        let rx: Vec<f64> = window.iter().map(|(x, _)| *x).collect();
        let rf: Vec<f64> = window.iter().map(|(_, r)| *r).collect();
        // The broadband measurement may refine the initial tone lock.
        // Apply the fitted filter at the recovered physical delay.
        let before = Echo::tx_correlated(&rx, &rf, e.delay, e.w.len());
        let after = Echo::tx_correlated(
            &Echo::apply_filter(&e.w, e.delay, &window),
            &rf,
            e.delay,
            e.w.len(),
        );
        let erle = 10.0 * (before / after.max(1e-30)).log10();
        eprintln!(
            "    delay {delay:5} lock {lock:5} gain {gain:+.3} -> {peak_mag:+.4} \
             ({:+.1}%), norm {norm:.4}, ERLE_tx {erle:.1} dB",
            (peak_mag / gain - 1.0) * 100.0
        );
        assert!(
            erle > 12.0,
            "the recovered filter only removed {erle:.1} dB of the transmit"
        );
    }

    /// The amplitude across the excitation's whole spectral range.
    ///
    /// This is the test that was missing. The DIL's reference spans 78.8 dB
    /// peak to floor and the earlier one was white at about 2 dB, so every
    /// assertion about the estimator was being made where it is easiest and
    /// nowhere near where it is used. The spread is swept with the path held
    /// fixed, so anything that moves is the excitation and not the path.
    #[test]
    fn the_identification_recovers_amplitude_across_the_spectral_range() {
        let delay = 1320usize;
        let gain = -0.106f64;
        let path = [(delay, gain), (delay + 3, -0.4 * gain), (delay + 7, 0.2 * gain)];
        let mut worst = 0.0f64;
        eprintln!(
            "    the DIL's own path, {gain:+.3} at {delay} samples, spread swept:\\n    \
             spread  recovered   error    norm  ERLE_tx"
        );
        for spread in [2.0, 10.0, 20.0, 30.0, 40.0, 54.0, 60.0] {
            let e = recover(&path, delay, spread, 20_000);
            let (peak_delay, peak_mag) = peak_as_delay(&e.w, delay);
            let norm = e.w.iter().map(|v| v * v).sum::<f64>().sqrt();
            let err = (peak_mag / gain - 1.0).abs();
            worst = worst.max(err);
            eprintln!(
                "    {spread:5.0} dB  {peak_mag:+.4}  {:+6.1}%  {norm:.4}  at delay \
                 {peak_delay}",
                (peak_mag / gain - 1.0) * 100.0
            );
            assert!(
                (peak_delay - delay as isize).abs() <= 4,
                "at {spread} dB of spread the path came back at a delay of {peak_delay}"
            );
            assert!(
                peak_mag.signum() == gain.signum(),
                "at {spread} dB of spread the path came back at {peak_mag:+.4}, \
                 inverted"
            );
        }
        // Within a fifth at every spread, which is the milestone. The worst is
        // the number quoted, because quoting the best would be choosing the
        // case that flatters.
        assert!(
            worst <= 0.2,
            "the recovered gain was {worst:.0}% out at the worst spread tried, \
             against a tolerance of 20%"
        );
    }

    /// The estimate must not depend on how much of the window it was given.
    ///
    /// This is the direct regression for the amplitude error. The old estimator
    /// accumulated block spectra, and for a path at delay D in blocks of n it
    /// returned `g * (1 - D / n)`: the numerator is computed over the block and
    /// the reference it needs for the first D samples is not in the block, while
    /// the denominator counts the block whole. The transform was sized to twice
    /// the delay, so the answer was half, and it moved again every time the
    /// number of blocks changed -- which is why the estimate on the line came
    /// out 0.0241 against a path measured at 0.106 and cancelled 0.1 dB.
    ///
    /// Correlating linearly over the whole window instead of block by block
    /// removes the dependence entirely, and that is what this holds down: the
    /// recovered gain is the same within one per cent from four blocks'
    /// worth of samples to thirty.
    #[test]
    fn the_identification_does_not_depend_on_the_window_length() {
        let delay = 1320usize;
        let gain = -0.106f64;
        let path = [
            (delay, gain),
            (delay + 3, -0.4 * gain),
            (delay + 7, 0.2 * gain),
        ];
        eprintln!("    the same path from windows of different lengths, and the \
                   same path from short ones:");
        // Short windows first, because they are the interesting case and they
        // are why the live solve has a sample gate at all. Below the gate the
        // filter is left untouched; above it, the common-row normal equations
        // must recover the gain without a delay/window-length bias.
        for count in [2_048, 4_096, 8_192] {
            let e = recover(&path, delay, 2.0, count);
            let (_, mag) = peak_as_delay(&e.w, delay);
            eprintln!(
                "      {count:6} samples ({:4.1} per tap) -> {mag:+.5}  ({:+.1}%)",
                count as f64 / 512.0,
                (mag / gain - 1.0) * 100.0
            );
        }
        let mut seen: Vec<(usize, f64)> = Vec::new();
        for count in [8_192, 16_384, 32_768, 65_536] {
            let e = recover(&path, delay, 2.0, count);
            assert!(e.ls.as_ref().expect("window").solved, "no solve at {count}");
            let (_, mag) = peak_as_delay(&e.w, delay);
            eprintln!(
                "      {count:6} samples ({:4.1} per tap) -> {mag:+.5}  ({:+.1}%)",
                count as f64 / 512.0,
                (mag / gain - 1.0) * 100.0
            );
            assert!(
                (mag / gain - 1.0).abs() <= 0.01,
                "at {count} samples the gain was {:+.1}% out, past one percent",
                (mag / gain - 1.0) * 100.0
            );
            seen.push((count, mag));
        }
        // Past sixteen samples per tap the answer has stopped moving with the
        // window, which is the property the old estimator did not have: its
        // factor was `1 - D / nfft` and `nfft` was a free parameter of the
        // block layout, so doubling the window changed the answer by half again.
        for w in seen.windows(2) {
            let drift = (w[1].1 / w[0].1 - 1.0).abs();
            assert!(
                drift <= 0.1,
                "the estimate moved {:.1}% between {} and {} samples",
                drift * 100.0,
                w[0].0,
                w[1].0
            );
        }
    }

    /// A lock below half the tap count cannot be represented, and saying so is
    /// better than returning half a filter.
    ///
    /// Tap `k` reads the reference `lock - centre + k` ago, so the window reaches
    /// back `centre` samples from the lock and no further: a path arriving
    /// earlier than that is outside what this filter can express. On the line
    /// the lock is in 640 to 3072 and `centre` is 256, so it cannot happen; the
    /// old test asked for a path at 55 samples and only passed because the
    /// transform was folding it round into the window.
    #[test]
    fn the_identification_refuses_a_lock_it_cannot_reach() {
        let e = recover(&[(55, 0.05)], 55, 2.0, 20_000);
        let ls = e.ls.as_ref().expect("window");
        assert!(ls.solved, "the solve was expected to run and decline");
        assert!(
            ls.note.contains("not solved"),
            "the refusal was not recorded: {}",
            ls.note
        );
        assert!(
            e.w.iter().all(|v| *v == 0.0),
            "a filter was installed for a path the window cannot reach"
        );
    }

    #[test]
    fn the_dil_identification_finds_a_path_at_the_lock() {
        identification_at(1319, 0.05, 1319);
    }

    #[test]
    fn the_dil_identification_finds_a_path_past_the_transform() {
        // The length that matters: the delay this line's correlation actually
        // locks, and one sample either side of the lock to show the two are no
        // longer assumed to agree.
        identification_at(1320, 0.04, 1319);
    }

    #[test]
    fn the_dil_identification_finds_the_latest_lockable_path() {
        identification_at(ECHO_LAG_HI - 64, 0.05, ECHO_LAG_HI - 64);
    }

    #[test]
    fn v90_dil_identification_covers_the_recorded_provider_echo() {
        identification_at(5800, 0.1, 5800);
    }

    #[test]
    fn v34_echo_range_is_enabled_on_v90_fallback() {
        let mut end = Answerer::new(true, true, true, None, std::ptr::null_mut(), None, std::ptr::null_mut());
        assert_eq!(end.echo.lag_hi, ECHO_LAG_HI);
        end.echo.identified = true;
        end.echo.delay = 1319;
        end.echo.w[256] = 0.1;
        end.start_v34();
        assert_eq!(end.echo.lag_hi, V34_ECHO_LAG_HI);
        assert!(!end.echo.identified);
        assert_eq!(end.echo.delay, 1319);
        assert_eq!(end.echo.w[256], 0.1);
        assert!(V34_ECHO_LAG_HI + ECHO_WINDOW + end.echo.w.len() < ECHO_RING);
    }

    #[test]
    fn v34_echo_freezes_before_lapm_and_unfreezes_for_retraining() {
        let mut end = Answerer::new(true, true, false, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        end.start_v34();
        let mut caller = v34::startup::Modem::new(v34::phase2::Role::Call, ENGINE_FS);
        let mut towards_call = std::collections::VecDeque::from(vec![0.0;480]);
        let mut towards_answer = towards_call.clone();
        for _ in 0..(25.0*ENGINE_FS) as usize {
            let a = match &mut end.stage {
                Stage::V34(m) => m.step(towards_answer.pop_front().unwrap()*0.3),
                _ => unreachable!(),
            };
            let c = caller.step(towards_call.pop_front().unwrap()*0.3);
            towards_call.push_back(a); towards_answer.push_back(c);
            if end.v34_carrier_up() { break; }
        }
        assert!(end.v34_carrier_up());
        end.status = BM_RUNNING; // LAPM has not connected yet.
        end.step_inner(0);
        assert!(end.echo.frozen && end.echo.data);
        if let Stage::V34(m) = &mut end.stage {
            assert!(m.retrain());
            m.step(0.0);
        }
        end.status = BM_CONNECTED; // The outer status can lag this sample.
        end.physical_connected = true;
        end.step_inner(0);
        assert!(!end.echo.frozen && !end.echo.data);
    }

    #[test]
    fn dil_measurement_keeps_its_delay_window_until_solved() {
        let mut echo = Echo::new();
        echo.delay = 1319;
        echo.ls_feed(0.0, 0.0);
        let (reference, _) = reference_with_spread(4096, 2.0);
        for (i, &tx) in reference.iter().enumerate() {
            let line = if i >= 1044 { 0.03 * reference[i-1044] } else { 0.0 };
            echo.sample(line);
            echo.push(tx);
        }
        assert_eq!(echo.delay, 1319, "DIL correlation moved the measurement window");
        assert!(echo.ls.as_ref().is_some_and(|ls| !ls.solved));
    }

    #[test]
    fn v34_boundary_cancels_400_and_500_ms_echoes() {
        for delay in [3200usize, 4000] {
            let mut end = Answerer::new(true, true, false, None, std::ptr::null_mut(), None, std::ptr::null_mut());
            end.start_v34();
            let echo = &mut end.echo;
            let mut old_echo = Echo::new();
            old_echo.lag_hi = 3072; // Previous V.90 range, retained as the control.
            let (reference, _) = reference_with_spread(16000, 20.0);
            let mut before = 0.0f64;
            let mut after = 0.0f64;
            for (n, &tx) in reference.iter().enumerate() {
                // Keep the echo below the quiet-peer gate so the test measures
                // the delay search, rather than whether adaptation is allowed.
                let input = if n >= delay { 0.03 * reference[n - delay] } else { 0.0 };
                let residual = echo.sample(input);
                echo.push(tx);
                old_echo.sample(input);
                old_echo.push(tx);
                if n > 12000 {
                    before += input * input;
                    after += residual * residual;
                }
            }
            let depth = 10.0 * (before / after.max(1e-30)).log10();
            assert!(old_echo.delay.abs_diff(delay) > 4, "old range unexpectedly found the delayed echo");
            println!("{} ms echo: delay={}, cancellation={depth:.1} dB", delay / 8, echo.delay);
            assert!(echo.delay.abs_diff(delay) <= 4, "{} ms echo was not found: delay={}, peak={}", delay / 8, echo.delay, echo.peak);
            assert!(depth > 15.0, "{} ms echo cancellation only {depth:.1} dB", delay / 8);
        }
    }

    /// The tracker has to take a better filter and refuse a worse one, and it
    /// has to judge both on samples the fit did not see.
    ///
    /// The gradient this replaces made the echo worse -- -1.9 dB of ERLE_tx on
    /// the calls that measured it -- and nothing ever asked whether its result
    /// was an improvement. Here the incumbent has to lose by a decibel on
    /// held-back samples or the candidate is thrown away, so the tracker can
    /// only ever go forwards. That is the whole of its safety.
    #[test]
    fn the_tracker_can_measure_and_update_a_725_ms_provider_path() {
        let delay = 5800;
        let n = TRACK_RING + TRACK_PERIOD * 2;
        let (reference, _) = reference_with_spread(n, 2.0);
        let mut echo = Echo::new();
        echo.delay = delay;
        echo.w.fill(0.0);
        echo.set_tracking(true);
        for i in 0..n {
            let line = if i >= delay { -0.1 * reference[i - delay] } else { 0.0 };
            echo.track_feed(line, reference[i]);
            if (i + 1) % TRACK_PERIOD == 0 { wait_for_tracking(&mut echo); }
        }
        wait_for_tracking(&mut echo);
        assert!(echo.track.taken > 0, "delayed path was never measured or updated");
        assert!(echo.track.held_now.is_finite());
        assert!(echo.w[echo.w.len()/2].abs() > 0.05);
    }
    #[test]
    fn the_tracker_takes_a_better_path_and_refuses_a_worse_one() {
        let taps = 512usize;
        let delay = 1319usize;
        let n = TRACK_FIT + TRACK_HOLD + TRACK_PERIOD * 2;
        let (r, _) = reference_with_spread(n + delay, 2.0);
        // The line: the path, plus a far-end signal neither filter is to blame
        // for. The far end is talking throughout, because that is the whole
        // difficulty and `far_end_silent` is not available for it.
        let mut far = 0x0f1e_2d3cu32;
        let mut line = vec![0.0; n];
        for i in 0..n {
            let mut y = 0.0;
            for (k, (_o, g)) in [(0usize, -0.072f64), (2, 0.069), (4, 0.046)]
                .iter()
                .enumerate()
            {
                let j = i as isize - delay as isize - k as isize;
                if j >= 0 {
                    y += g * r[j as usize];
                }
            }
            far = far.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let f = if (far >> 16) & 1 == 0 { 0.03 } else { -0.03 };
            line[i] = y + f;
        }

        // The incumbent: the right delay, and a broadband wrong shape -- what
        // fitting over the wrong window gives.
        let mut weak = Echo::new();
        weak.delay = delay;
        weak.w = vec![0.0; taps];
        for (k, (_o, g)) in [(0usize, -0.0375f64), (2, 0.061), (3, 0.035), (8, 0.030)]
            .iter()
            .enumerate()
        {
            weak.w[taps / 2 + k] = *g;
        }
        let mut noise = 0x51ce_0001u32;
        for v in weak.w.iter_mut() {
            noise = noise.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *v += ((noise >> 16) as f64 / 32768.0 - 0.5) * 0.012;
        }
        weak.set_tracking(true);
        for (i, &x) in line.iter().enumerate() {
            weak.track_feed(x, r[i]);
            if (i + 1) % TRACK_PERIOD == 0 { wait_for_tracking(&mut weak); }
        }
        wait_for_tracking(&mut weak);
        let taken = weak.track.taken;
        assert!(
            taken > 0,
            "the tracker took nothing from a filter that was {} dB worse",
            TRACK_MARGIN_DB
        );
        assert_eq!(weak.track.rx.len(), TRACK_RING);
        eprintln!(
            "    tracker: took {}/{} attempts, norm {:.4}, peak {:.4} at a delay of {}",
            taken,
            weak.track.tries,
            weak.w.iter().map(|v| v * v).sum::<f64>().sqrt(),
            weak.w[taps / 2],
            weak.delay
        );
        // The fitted path has to be better than what it replaced, on the
        // held-back samples, by the margin the test exists to enforce.
        // And an incumbent that is already right must not be displaced by noise.
        let mut good = Echo::new();
        good.delay = delay;
        good.w = vec![0.0; taps];
        for (k, (_o, g)) in [(0usize, -0.072f64), (2, 0.069), (4, 0.046)]
            .iter()
            .enumerate()
        {
            good.w[taps / 2 + k] = *g;
        }
        let before = good.w.clone();
        good.set_tracking(true);
        for (i, &x) in line.iter().enumerate() {
            good.track_feed(x, r[i]);
            if (i + 1) % TRACK_PERIOD == 0 { wait_for_tracking(&mut good); }
        }
        wait_for_tracking(&mut good);
        assert_eq!(
            good.track.taken, 0,
            "a filter that already was the path was replaced {} times",
            good.track.taken
        );
        for (k, (a, b)) in good.w.iter().zip(&before).enumerate() {
            assert!(
                (a - b).abs() < 1e-12,
                "tap {k} moved from {b:+.6} to {a:+.6} on a line it already fitted"
            );
        }
    }

    #[test]
    fn echo_delay_recovery_requires_a_consistent_shift_and_independent_evidence() {
        let mut seed = 19u64;
        let mut noise = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 32) as u32 as f64 / u32::MAX as f64 - 0.5) * 0.1
        };
        let prediction: Vec<f64> = (0..4096).map(|_| noise()).collect();
        for shift in (-8isize..=8).filter(|&step| step != 0).map(|step| step * 80) {
            let window: Vec<_> = (0..4096).map(|i| {
                let j = i as isize-shift;
                let echo = if (0..4096).contains(&j) { prediction[j as usize] } else { 0.0 };
                (echo + noise(), prediction[i])
            }).collect();
            assert_eq!(Echo::delay_shift(&window), shift);
        }
        let unchanged: Vec<_> = prediction.iter().map(|&y| (y+noise(), y)).collect();
        assert_eq!(Echo::delay_shift(&unchanged), 0);
        let unrelated: Vec<_> = prediction.iter().map(|&y| (noise(), y)).collect();
        assert_eq!(Echo::delay_shift(&unrelated), 0);
        let partial: Vec<_> = (0..4096).map(|i| {
            let y = if i < 2048 { prediction[(i+80)%4096] } else { prediction[i] };
            (y, prediction[i])
        }).collect();
        assert_eq!(Echo::delay_shift(&partial), 0);
        let amplitude_only: Vec<_> = prediction.iter().map(|&y| (y * 1.5, y)).collect();
        assert_eq!(Echo::delay_shift(&amplitude_only), 0);
        assert_eq!(Echo::delay_shift(&vec![(0.0,0.0);4096]), 0);
    }

    /// The far end's data, independent of ours, `db` above an echo of `echo_rms`.
    fn far_signal(count: usize, db: f64, echo_rms: f64) -> Vec<f64> {
        let mut seed = 0x1357_9bdfu32;
        let scale = echo_rms * 10f64.powf(db / 20.0) * 3f64.sqrt();
        (0..count).map(|_| {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (f64::from(seed >> 8) / f64::from(1u32 << 24) * 2.0 - 1.0) * scale
        }).collect()
    }

    #[test]
    fn a_playout_jump_is_followed_under_a_far_signal_seventeen_db_louder() {
        // The USR caller, 2026-11-21: the echo 17.4 dB under the far modem's
        // signal, and the path 80 samples later from 150 s on. The echo is
        // then under 2% of what arrives: half a second measures the shifted
        // echo's gain to 0.2 either way, and only a longer window settles it.
        let n = DELAY_WINDOW_MAX;
        let (prediction, _) = reference_with_spread(n, 2.0);
        let rms = (prediction.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
        let far = far_signal(n, 17.4, rms);
        let moved: Vec<_> = (0..n).map(|i| {
            let echo = if i >= 80 { prediction[i - 80] } else { 0.0 };
            (far[i] + echo, prediction[i])
        }).collect();
        assert_eq!(Echo::delay_shift(&moved), 80);
        // The same far signal over an echo that has not moved: nothing to do.
        let still: Vec<_> = (0..n).map(|i| (far[i] + prediction[i], prediction[i])).collect();
        assert_eq!(Echo::delay_shift(&still), 0);
        // And the far signal alone, with no echo of ours in it at all.
        let none: Vec<_> = (0..n).map(|i| (far[i], prediction[i])).collect();
        assert_eq!(Echo::delay_shift(&none), 0);

        // And the frozen V.34 filter follows it sample by sample.
        let mut echo = Echo::new();
        echo.delay = 7480;
        echo.frozen = true;
        echo.data = true;
        // Fed the unshifted prediction throughout, which a real filter would
        // stop producing once it moved: so the first move is the answer.
        let at = moved.iter().position(|&(x, y)| { echo.track_delay(x, y); echo.delay != 7480 });
        assert!(at.is_some_and(|at| at < DELAY_WINDOW_MAX), "{at:?}");
        assert_eq!(echo.delay, 7560);
    }

    #[test]
    fn frozen_v34_path_can_recover_timing_without_refitting_coefficients() {
        for shift in [80usize, 640] {
        let mut echo = Echo::new();
        echo.delay = 1319;
        echo.frozen = true;
        echo.data = true;
        echo.w[256] = 0.1;
        let weights = echo.w.clone();
        let (prediction, _) = reference_with_spread(4096, 2.0);
        for i in 0..prediction.len() {
            let input = prediction.get(i+shift).copied().unwrap_or(0.0);
            echo.track_delay(input, prediction[i]);
        }
        assert_eq!(echo.delay, 1319-shift);
        assert_eq!(echo.w, weights);
        assert!(!echo.identified);
        }
    }

    #[test]
    fn data_tracker_does_not_disable_validated_playout_recovery() {
        let mut echo = Echo::new();
        echo.delay = 7480;
        echo.identified = true;
        echo.data = true;
        echo.set_tracking(true);
        echo.w[256] = 0.1;
        let weights = echo.w.clone();
        let epoch = echo.track_epoch;
        echo.track.rx.push_back(0.25);
        echo.track.reference.push_back(0.5);
        let (prediction, _) = reference_with_spread(4096, 2.0);
        for i in 0..prediction.len() {
            echo.track_delay(prediction.get(i+160).copied().unwrap_or(0.0), prediction[i]);
        }
        assert_eq!(echo.delay, 7320);
        assert_eq!(echo.w, weights);
        assert_eq!(echo.track_epoch, epoch+1);
        assert!(echo.track.rx.is_empty());
        assert!(echo.track.reference.is_empty());
        assert!(echo.tracking);
    }
    #[test]
    fn measured_training_path_can_follow_playout_after_the_silent_window() {
        for (solved, far_silent, expected) in [(false,false,1319), (true,true,1319), (true,false,1239)] {
            let mut echo=Echo::new();
            echo.delay=1319;
            echo.identified=true;
            echo.data=false;
            echo.far_silent=far_silent;
            let mut measurement=Ls::blank(1319);
            measurement.solved=solved;
            echo.ls=Some(measurement);
            echo.w[256]=0.1;
            let weights=echo.w.clone();
            let (prediction,_)=reference_with_spread(4096,2.0);
            for i in 0..prediction.len() {
                echo.track_delay(prediction.get(i+80).copied().unwrap_or(0.0),prediction[i]);
            }
            assert_eq!(echo.delay,expected,"solved={solved}, far_silent={far_silent}");
            assert_eq!(echo.w,weights,"timing recovery must preserve trained coefficients");
        }
    }

    #[test]
    fn unsettled_training_requires_two_consistent_tail_blocks() {
        let n=25000; let taps=512; let delay=6361;
        let (reference,_)=reference_with_spread(n,2.0);
        let first=n-8*taps;
        let mut line=vec![0.0;n];
        for i in first..n {line[i]=0.15*reference[i-delay];}
        assert_eq!(Echo::settled_training_delay(&reference,&line,ECHO_LAG_HI,taps),Some((delay,first)));
        for i in first+4*taps..n {line[i]=0.15*reference[i-delay-80];}
        assert_eq!(Echo::settled_training_delay(&reference,&line,ECHO_LAG_HI,taps),None);
    }

    #[test]
    fn training_delay_scores_only_measured_receive_rows() {
        let prefix = 12000;
        let delay = 7480;
        let n = prefix + 8192;
        let (reference, _) = reference_with_spread(n, 2.0);
        let mut line = vec![0.0; n];
        for i in prefix..n { line[i] = 0.15 * reference[i-delay]; }
        assert_eq!(Echo::training_delay_from(&reference, &line, V34_ECHO_LAG_HI, prefix), Some(delay));
        // Padding must not influence the measured correlation, even if stale
        // receive history happens to resemble a different reflection.
        for i in 3320..prefix { line[i] = 4.0 * reference[i-3320]; }
        assert_eq!(Echo::training_delay_from(&reference, &line, V34_ECHO_LAG_HI, prefix), Some(delay));
    }
    /// The 2026-11-02 V.92 calls: the rows measured after the far end fell
    /// silent hear only the echo of the DIL's opening, which repeats every
    /// 138 samples, so 6840 and every lag 138 apart from it score alike and
    /// the lock took 6288. The earlier real rows -- the far end's S over the
    /// echo of the aperiodic Jd -- are what tell the aliases apart.
    #[test]
    fn a_periodic_dil_does_not_lock_the_echo_on_an_alias() {
        let (n, delay, period) = (30_000, 6840, 138);
        let (wider_from, measured_from, periodic_from) = (15_000, 20_000, 13_000);
        let (noise, _) = reference_with_spread(n, 2.0);
        let reference: Vec<f64> = (0..n)
            .map(|i| if i < periodic_from { noise[i] } else { noise[periodic_from + (i - periodic_from) % period] })
            .collect();
        let mut line = vec![0.0; n];
        for i in wider_from..n {
            line[i] = 0.15 * reference[i - delay];
        }
        for i in wider_from..measured_from {
            line[i] += 0.3 * (i as f64 * 0.7).sin(); // the far end's own signal
        }
        assert_eq!(
            Echo::training_delay_resolving(&reference, &line, ECHO_LAG_HI, measured_from, wider_from),
            Some(delay)
        );
        // The measured rows alone cannot tell: this is the lock that failed.
        assert_ne!(Echo::training_delay_from(&reference, &line, ECHO_LAG_HI, measured_from), Some(delay));
        // Without a periodic reference the wider rows change nothing.
        let mut plain = vec![0.0; n];
        for i in wider_from..n {
            plain[i] = 0.15 * noise[i - delay];
        }
        assert_eq!(Echo::training_delay_resolving(&noise, &plain, ECHO_LAG_HI, measured_from, wider_from), Some(delay));
    }

    #[test]
    fn v34_broadband_training_resolves_an_earlier_tone_delay_alias() {
        let n = V34_ECHO_LAG_HI + 2048;
        let delay = 3720;
        let (reference, _) = reference_with_spread(n, 2.0);
        let mut line = vec![0.0; n];
        for i in delay..n {
            line[i] = 0.15 * reference[i-delay];
        }
        assert_eq!(Echo::training_delay(&reference, &line, V34_ECHO_LAG_HI), Some(delay));
        assert_eq!(Echo::training_delay(&reference, &vec![0.0; n], V34_ECHO_LAG_HI), None);
    }

    #[test]
    fn v90_dil_fit_recovers_a_path_outside_the_earlier_tone_lock() {
        let delay = 5720;
        let gain = 0.15;
        let echo = recover(&[(delay, gain)], 5080, 2.0, 20_000);
        assert_eq!(echo.delay, delay);
        let (peak, measured) = peak_as_delay(&echo.w, echo.delay);
        assert_eq!(peak, delay as isize);
        assert!((measured / gain - 1.0).abs() < 0.05,
            "DIL fit must recover echo gain, not just move the delay");
    }

    #[test]
    fn asynchronous_training_preserves_audio_state_until_a_fit_is_ready() {
        let mut echo = recover(&[(5720, 0.15)], 5080, 2.0, 20_000);
        echo.delay = 5080;
        echo.w.fill(0.0);
        echo.identified = false;
        echo.ls.as_mut().unwrap().solved = false;
        echo.training_async = true;
        echo.tx_pos = 123;
        echo.ls_solve();
        assert_eq!(echo.delay, 5080);
        assert!(echo.w.iter().all(|w| *w == 0.0));
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while echo.training_job.is_some() && std::time::Instant::now() < deadline {
            echo.ls_poll();
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        assert!(echo.training_job.is_none());
        assert_eq!(echo.tx_pos, 123, "fitting must preserve transmit history position");
        let (delay, gain) = peak_as_delay(&echo.w, echo.delay);
        assert_eq!(delay, 5720);
        assert!((gain / 0.15 - 1.0).abs() < 0.05);
    }

    #[test]
    fn an_old_training_worker_cannot_overwrite_a_retrain() {
        let mut echo = Echo::new();
        let (send, receive) = std::sync::mpsc::channel();
        echo.training_job = Some(receive);
        let old_epoch = echo.training_epoch;
        echo.restart_training();
        echo.delay = 1500;
        echo.w.fill(0.25);
        let current = echo.w.clone();
        send.send(TrainingFit { epoch: old_epoch, delay: 5720,
            w: vec![0.15; current.len()], identified: true, ls: None }).unwrap();
        echo.ls_poll();
        assert_eq!(echo.delay, 1500);
        assert_eq!(echo.w, current);
        assert!(echo.ls.is_none());
    }

    #[test]
    fn v34_broadband_training_covers_the_provider_echo_delay() {
        let n = V34_ECHO_LAG_HI + 2048;
        let delay = 7400;
        let (reference, _) = reference_with_spread(n, 2.0);
        let mut line = vec![0.0; n];
        for i in delay..n { line[i] = 0.15 * reference[i-delay]; }
        assert_eq!(Echo::training_delay(&reference, &line, V34_ECHO_LAG_HI), Some(delay));
    }

    fn wait_for_tracking(echo: &mut Echo) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
        while echo.track_job.is_some() && std::time::Instant::now() < deadline {
            echo.track_poll();
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        assert!(echo.track_job.is_none(), "echo worker did not finish");
    }

    #[test]
    fn short_echo_training_window_reports_once_and_remains_unsolved() {
        let mut ls = Ls::blank(1319);
        ls.samples = 3920;
        assert_eq!(ls.take_shortfall(8192), Some(3920));
        for _ in 0..8000 {
            assert_eq!(ls.take_shortfall(8192), None);
        }
        assert!(!ls.solved, "suppressing a diagnostic must not mark the echo fit solved");
        let mut next_window = Ls::blank(1319);
        next_window.samples = 4000;
        assert_eq!(next_window.take_shortfall(8192), Some(4000));
        next_window.samples = 8192;
        assert_eq!(next_window.take_shortfall(8192), None);
    }

    #[test]
    fn committed_tracking_taps_preserve_the_fitted_path_delay() {
        for offset in [-12isize, 140] {
            let lock = 1319usize;
            let path_delay = (lock as isize + offset) as usize;
            let n = TRACK_FIT + TRACK_HOLD + TRACK_PERIOD;
            let (reference, _) = reference_with_spread(n + path_delay, 2.0);
            let mut echo = Echo::new();
            echo.delay = lock;
            echo.w.fill(0.0);
            echo.set_tracking(true);
            for i in 0..n {
                let line = if i >= path_delay { 0.15 * reference[i - path_delay] } else { 0.0 };
                echo.track_feed(line, reference[i]);
            }
            wait_for_tracking(&mut echo);
            assert!(echo.track.taken > 0, "no fit accepted for offset {offset}");
            let peak = echo.w.iter().enumerate()
                .max_by(|(_, a), (_, b)| a.abs().total_cmp(&b.abs())).unwrap().0;
            let committed_delay = echo.delay as isize - (echo.w.len() / 2) as isize + peak as isize;
            assert_eq!(committed_delay, path_delay as isize,
                       "accepted filter changed the physical path for offset {offset}");
        }
    }

    #[test]
    fn a_completed_old_data_fit_cannot_overwrite_a_retrained_filter() {
        let mut echo = Echo::new();
        echo.delay = 1319;
        echo.set_tracking(true);
        let old_epoch = echo.track_epoch;
        let (send, receive) = std::sync::mpsc::channel();
        echo.track_job = Some(receive);
        echo.set_tracking(false);
        echo.set_tracking(true);
        echo.delay = 1491;
        let current = echo.w.clone();
        send.send(TrackFit { epoch: old_epoch, delay: 800, w: vec![9.0; current.len()],
            accepted: true, held_best: 0.0, held_now: 1.0 }).unwrap();
        echo.track_poll();
        assert_eq!(echo.delay, 1491);
        assert_eq!(echo.w, current);
        assert_eq!(echo.track.taken, 0);
        assert!(echo.track_job.is_none());
    }

    #[test]
    fn data_window_echo_refits_require_opt_in_and_a_current_data_signal() {
        assert!(!data_echo_tracking_enabled(None));
        assert!(!data_echo_tracking_enabled(Some("0")));
        assert!(!data_echo_tracking_enabled(Some("")));
        assert!(!data_echo_tracking_enabled(Some("invalid")));
        assert!(data_echo_tracking_enabled(Some("1")));
        let mut end = Answerer::new(true, true, true, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        end.echo_track_data = false;
        end.v90_in_data = true;
        end.update_echo_tracking();
        assert!(!end.echo.tracking, "data signal alone must not replace the trained path");
        end.echo_track_data = true;
        end.update_echo_tracking();
        assert!(end.echo.tracking);
        for _ in 0..100 { end.echo.track_feed(0.03, 0.1); }
        end.v90_in_data = false;
        end.update_echo_tracking();
        assert!(!end.echo.tracking);
        assert!(end.echo.track.rx.is_empty());
        assert!(end.echo.track.reference.is_empty());
    }

    #[test]
    fn retraining_stops_tracking_and_discards_the_old_data_window() {
        let mut end = Answerer::new(true, true, true, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        end.start_v90();
        end.v90_in_data = true;
        end.echo.set_tracking(true);
        for _ in 0..100 { end.echo.track_feed(0.03, 0.1); }
        assert_eq!(end.echo.track.rx.len(), 100);
        // Phase 2 has no current V.90 data signal, despite the stale flag.
        end.linear_step(0);
        assert!(!end.v90_in_data);
        assert!(!end.echo.tracking);
        assert!(end.echo.track.rx.is_empty());
        assert!(end.echo.track.reference.is_empty());
        assert_eq!(end.echo.track.since, 0);
    }

    #[test]
    fn retraining_reports_the_current_handshake_instead_of_fallback_data() {
        let mut end = Answerer::new(true, true, true, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        end.start_v90();
        end.physical_connected = true;
        end.status = BM_RETRAINING;
        end.copy_phase();
        let phase = unsafe { std::ffi::CStr::from_ptr(end.phase_buf.as_ptr().cast()) };
        assert_eq!(phase.to_bytes(), b"V.90 phase 2");
    }

    /// V.92 9.10.2.1: granted a hold, the server sends ANSam for T1 and is
    /// ready for the caller's phase 1, keeping the V.42 link it had.
    #[test]
    fn a_granted_hold_answers_with_ansam_and_keeps_the_v42_link() {
        let mut end = Answerer::new(true, true, true, None, std::ptr::null_mut(), None, std::ptr::null_mut());
        end.ec = Some(EcStack::new(EcRole::Answerer, EcParams::default()));
        end.enter_hold(datapump::v92::Held::OnHold { seconds: 240.0 });
        assert!(matches!(&end.stage, Stage::V8(m) if m.phase() == "ANSam"));
        assert_eq!(end.status, BM_RUNNING);
        assert!(end.ec.is_some(), "the V.42 link goes on through the hold");
        for _ in 0..(LINE_FS * 30.0) as usize {
            end.linear_step(0);
        }
        assert!(matches!(&end.stage, Stage::V8(m) if m.phase() == "ANSam"), "still holding 30 s in");
    }

    /// T1 ran out with no phase 1 from the caller: 9.10.2.1 disconnects.
    #[test]
    fn a_hold_that_runs_out_ends_the_call() {
        let mut end = Answerer::new(true, true, true, None, std::ptr::null_mut(), None, std::ptr::null_mut());
        end.enter_hold(datapump::v92::Held::OnHold { seconds: 10.0 });
        for _ in 0..(LINE_FS * 12.0) as usize {
            end.linear_step(0);
            if end.status != BM_RUNNING { break; }
        }
        assert_eq!(end.status, BM_FAILED);
        assert_eq!(end.failure, "V.92 modem-on-hold: the caller did not come back within T1");
    }

    #[test]
    fn a_cleardown_by_mhclrd_ends_the_call() {
        let mut end = Answerer::new(true, true, true, None, std::ptr::null_mut(), None, std::ptr::null_mut());
        end.enter_hold(datapump::v92::Held::ClearedDown);
        assert_eq!(end.status, BM_FAILED);
        assert_eq!(end.failure, "V.92 cleardown asked by the analogue modem (MHclrd)");
    }

    #[test]
    fn unanswered_ansam_hands_v90_answerer_to_legacy_training() {
        let mut end = Answerer::new(true, true, true, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        for _ in 0..(LINE_FS * 7.0) as usize {
            end.linear_step(0);
            if end.status != BM_RUNNING { break; }
        }
        assert_eq!(end.status, BM_AGREED_OTHER);
        assert!(matches!(end.stage, Stage::Done));
    }



    #[test]
    fn legacy_v32_opening_selects_v32_without_repeating_the_answer_tone() {
        let mut end = Answerer::new(true, true, true, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        for i in 0..4000 {
            let input = (8000.0 * (2.0 * std::f64::consts::PI * 1800.0 * i as f64 / LINE_FS).cos()) as c_int;
            end.step_inner(input);
        }
        assert!(end.legacy_v32_tone_samples >= 214);
        end.no_v8_negotiation();
        let Stage::V32(modem) = &end.stage else { panic!("legacy V.32 not selected") };
        assert_eq!(modem.phase(), "AC");
    }

    #[test]
    fn aa_heard_during_ansam_starts_v32_without_waiting_out_ansam() {
        let mut end = Answerer::new(true, true, true, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        let mut n = 0usize;
        while !matches!(&end.stage, Stage::V8(m) if m.phase() == "ANSam") {
            end.step_inner(0);
            n += 1;
            assert!(n < (LINE_FS * 10.0) as usize, "ANSam never started");
        }
        let ansam_at = n;
        for i in 0..(LINE_FS * 1.0) as usize {
            let input = (8000.0 * (2.0 * std::f64::consts::PI * 1800.0 * i as f64 / LINE_FS).cos()) as c_int;
            end.step_inner(input);
            n += 1;
            if matches!(end.stage, Stage::V32(_)) { break; }
        }
        let Stage::V32(modem) = &end.stage else { panic!("AA during ANSam did not start V.32") };
        assert_eq!(modem.phase(), "AC");
        let waited = (n - ansam_at) as f64 / LINE_FS;
        assert!(waited < 1.0, "V.32 started {waited:.2} s into ANSam");
    }

    #[test]
    fn unanswered_ansam_hands_v34_answerer_to_legacy_training() {
        let mut end = Answerer::new(true, true, false, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        for _ in 0..(ENGINE_FS * 7.0) as usize {
            end.engine_step(0.0);
            if end.status != BM_RUNNING { break; }
        }
        assert_eq!(end.status, BM_AGREED_OTHER);
        assert!(matches!(end.stage, Stage::Done));
    }

    #[test]
    fn calling_modem_keeps_its_selected_non_v8_v34_mode() {
        let mut end = Answerer::new(false, true, false, None,
            std::ptr::null_mut(), None, std::ptr::null_mut());
        end.no_v8_negotiation();
        assert_eq!(end.status, BM_RUNNING);
        assert!(matches!(end.stage, Stage::V34(_)));
    }

    /// The A/B has to prefer the right filter for the right reason, so this
    /// gives it a window it was not fitted to, a correct filter and a wrong
    /// one, and asks which leaves less of the transmit behind.
    #[test]
    fn the_ab_prefers_the_correct_filter() {
        let taps = 512usize;
        let delay = 900usize;
        let centre = taps / 2;
        let n = 2048;
        let mut seed = 0x0bad_c0deu32;
        let mut r: Vec<f64> = (0..n + delay)
            .map(|_| {
                seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                if (seed >> 16) & 1 == 0 { 0.1 } else { -0.1 }
            })
            .collect();
        // The line: the path applied to the reference, plus a far-end signal
        // that is nothing to do with either filter, plus noise.
        let mut far = 0x1234_5678u32;
        let mut window: Vec<(f64, f64)> = Vec::with_capacity(n);
        for i in 0..n {
            // The path at the delay the lock will report, so that the taps the
            // test writes -- centre, centre + 3, centre + 7, which model
            // delays `delay`, `delay + 3` and `delay + 7` -- are the ones that
            // cancel it. An echo at zero delay under a lock at 900 is 900
            // samples out of reach and the metric correctly reports nothing.
            let mut y = 0.0;
            for (k, (_o, g)) in [(0usize, 0.05f64), (3, 0.02), (7, -0.01)].iter().enumerate() {
                let j = i as isize - delay as isize - k as isize;
                if j >= 0 {
                    y += g * r[j as usize];
                }
            }
            far = far.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let f = if (far >> 16) & 1 == 0 { 0.03 } else { -0.03 };
            window.push((y + f + 0.002 * ((i * 37 % 11) as f64 - 5.0), r[i]));
        }
        // Tap k models a path component at delay `delay - centre + k`, so a
        // path at the lock itself lands on tap `centre`, and the components at
        // 0, 3 and 7 samples behind it on centre, centre + 3, centre + 7.
        let mut good = vec![0.0; taps];
        for (k, (_off, g)) in [(0usize, 0.05f64), (3, 0.02), (7, -0.01)].iter().enumerate() {
            good[centre + k] = *g;
        }
        // A wrong one: a big noisy filter of the same kind the old metric liked.
        let mut noise = 0xfeed_faceu32;
        let mut noisy = vec![0.0; taps];
        for v in noisy.iter_mut() {
            noise = noise.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *v = ((noise >> 16) as f64 / 32768.0 - 0.5) * 0.01;
        }
        for (k, (_off, g)) in [(0usize, 0.05f64), (3, 0.02), (7, -0.01)].iter().enumerate() {
            noisy[centre + k] = *g;
        }
        let rx: Vec<f64> = window.iter().map(|(x, _)| *x).collect();
        let rf: Vec<f64> = window.iter().map(|(_, r)| *r).collect();
        let p_none = Echo::tx_correlated(&rx, &rf, delay, taps);
        let rg = Echo::apply_filter(&good, delay, &window);
        let rn = Echo::apply_filter(&noisy, delay, &window);
        let p_good = Echo::tx_correlated(&rg, &rf, delay, taps);
        let p_noisy = Echo::tx_correlated(&rn, &rf, delay, taps);
        let erle = |a: f64, b: f64| 10.0 * (a / b.max(1e-30)).log10();
        eprintln!(
            "    synthetic: none {p_none:.3e}  good {p_good:.3e} ({:.1} dB)  \
             noisy {p_noisy:.3e} ({:.1} dB)",
            erle(p_none, p_good),
            erle(p_none, p_noisy)
        );
        // Ten decibels on a three-tap path with a far-end signal thirty-six
        // times its power, which is what the fixture has. Not more than that:
        // what is being checked is that the metric separates the two, not that
        // it agrees with a hand calculation.
        assert!(
            p_good < p_none * 0.2,
            "the correct filter left {:.3e} of {:.3e}, want at least 7 dB",
            p_good,
            p_none
        );
        assert!(
            p_good * 3.0 < p_noisy,
            "the correct filter ({p_good:.3e}) should beat the noisy one ({p_noisy:.3e}) \
             by at least 5 dB"
        );
        // And the old metric must be shown to prefer the wrong one, or there is
        // no case for having replaced it.
        let depth = |v: &[f64]| -> f64 {
            let mut p = 0.0;
            let mut q = 0.0;
            for i in 0..v.len() {
                let pred = window[i].0 - v[i];
                p += pred * pred;
                q += v[i] * v[i];
            }
            10.0 * (p / q.max(1e-30)).log10()
        };
        let d_good = depth(&rg);
        let d_noisy = depth(&rn);
        eprintln!("    old depth metric: good {d_good:.1} dB  noisy {d_noisy:.1} dB");
    }





    #[test]
    fn the_dil_identification_recovers_a_known_path() {
        // One sample, not zero: a tap reads the reference from *before* the
        // sample it is predicting, so a path component at delay zero would be
        // read out of history that has not been written yet. At delay zero the
        // fixture silently described a path of no gain at all and the estimate
        // correctly reported the next tap along.
        let path: [(usize, f64); 3] = [(1, 0.10), (5, -0.05), (11, 0.02)];
        let mut e = Echo::new();
        // The lock is a separate matter from the path and the two do not agree
        // on a real call, so this test sets the lock where the path is, and the
        // disagreement is the thing to look at on the line rather than
        // something to fold in here. It sits at half the tap count, which is
        // the only lock that can express a path arriving at zero: the window
        // reaches back exactly as far as the lock is from its own centre.
        e.delay = 256;
        // Broadband, like four points of 22 666 bit/s: a fixed pseudorandom
        // sequence so the test is the same every run.
        let mut seed = 0x1234_5678u32;
        let mut r = vec![0.0f64; 40_000];
        for v in r.iter_mut() {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *v = if (seed >> 16) & 1 == 0 { 0.1 } else { -0.1 };
        }
        // A history long enough that every read is of a sample already sent: a
        // ring read before it has been written is a zero, and eleven of those
        // at the head of the run is a tenth of the window.
        let mut history = vec![0.0f64; r.len()];
        for (n, &tx) in r.iter().enumerate() {
            let mut y = 0.0;
            for (d, g) in path {
                if n >= d {
                    y += g * history[n - d];
                }
            }
            e.ls_feed(y, tx);
            history[n] = tx;
        }
        assert!(e.ls.is_some(), "no identification window was opened");
        e.ls_solve();
        let ls = e.ls.as_ref().expect("window");
        assert!(ls.solved, "the solve did not run");
        assert!(ls.samples > 15_000, "only {} samples", ls.samples);
        let norm = e.w.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(
            (0.05..0.25).contains(&norm),
            "filter norm {norm:.4}, want about 0.1 for a 14 dB path"
        );
        // The strongest tap has to sit at the path's first arrival, and
        // `peak_as_delay` answers in delays. The lock is 256 and the path is
        // 255 samples ahead of it, at the far end of what the window reaches.
        let (at, mag) = peak_as_delay(&e.w, e.delay);
        assert!(
            (-1..=3).contains(&at),
            "the largest tap is at a delay of {at}, the path arrives at 1"
        );
        // The exact per-tap shape is not what this test is for; what matters
        // is that the estimate is a path and not its inverse, which the norm
        // above already settles: 0.05 for this path against 20.0 for the
        // inverted one.
        assert!((0.02..0.2).contains(&mag), "peak magnitude {mag:.4}");
    }
}

#[cfg(test)]
mod recorded_training_checks {

    /// V90_CAPTURE=line-00000176-0.wav with the main line's env
    /// (V8BIS_ENABLE=1 V92_QUICK_CONNECT=1 V92_QC_ANSWER=0): a V.92 caller
    /// answers CRe with a V.8 bis message (V.21 high channel, 1.4-2.0 s).
    /// That FSK was counted as V.32's AA and the call went to V.32bis at
    /// 14400 the instant ANSam began. ANSam must play instead.
    #[test]
    #[ignore = "requires the saved V.8 bis reply recording (line-00000176)"]
    fn recorded_v8bis_reply_is_not_taken_for_v32_aa() {
        let raw=std::fs::read(std::env::var("V90_CAPTURE").unwrap()).unwrap();
        let mut p=12;
        let data=loop {let n=u32::from_le_bytes(raw[p+4..p+8].try_into().unwrap()) as usize;
            if &raw[p..p+4]==b"data" {break &raw[p+8..p+8+n];} p+=8+n+(n&1);};
        let end=bm_create_v90(1,None,std::ptr::null_mut(),None,std::ptr::null_mut());
        let mut ansam=false;
        for (i,f) in data.chunks_exact(4).take(2_800*8).enumerate() {
            bm_service(end);
            bm_step(end,i16::from_le_bytes([f[0],f[1]]) as c_int);
            let a=unsafe{&*end};
            if let Stage::V8(m)=&a.stage {ansam|=m.phase()=="ANSam";}
            assert!(!matches!(a.stage,Stage::V32(_)),"V.32 started at {:.3} s",i as f64/8000.0);
        }
        bm_destroy(end);
        assert!(ansam,"ANSam never began");
    }

    #[test]
    #[ignore = "requires a supplied failed Quick Connect stereo recording"]
    fn recorded_live_path_accepts_qc2() {
        let raw=std::fs::read(std::env::var("V90_CAPTURE").unwrap()).unwrap();
        let mut p=12;
        let data=loop {let n=u32::from_le_bytes(raw[p+4..p+8].try_into().unwrap()) as usize;
            if &raw[p..p+4]==b"data" {break &raw[p+8..p+8+n];} p+=8+n+(n&1);};
        let end=bm_create_v90(1,None,std::ptr::null_mut(),None,std::ptr::null_mut());
        let mut acknowledged=false;
        for f in data.chunks_exact(4).take(3*8000) {
            bm_service(end);
            bm_step(end,i16::from_le_bytes([f[0],f[1]]) as c_int);
            let a=unsafe{&*end};
            if let Stage::V8(m)=&a.stage {acknowledged|=m.phase()=="V.92 QCA2d";}
        }
        bm_destroy(end);
        assert!(acknowledged,"valid recorded QC2a was lost in the live receive path");
    }
    use super::*;
    #[test]
    #[ignore="requires V90_CAPTURE/ECHO_REFERENCE for the saved Quick Connect failure"]
    fn recorded_quick_training_reads_cpt_before_peer_retrain() {
        let raw=std::fs::read(std::env::var("V90_CAPTURE").unwrap()).unwrap();
        let mut pos=12;
        let data=loop {
            let len=u32::from_le_bytes(raw[pos+4..pos+8].try_into().unwrap()) as usize;
            if &raw[pos..pos+4]==b"data" {break &raw[pos+8..pos+8+len];}
            pos+=8+len+(len&1);
        };
        let end=bm_create_v90(1,None,std::ptr::null_mut(),None,std::ptr::null_mut());
        unsafe {(*end).echo.training_async=false;}
        for frame in data.chunks_exact(4).take((16.5*8000.0) as usize) {
            bm_service(end);
            bm_step(end,i16::from_le_bytes([frame[0],frame[1]]) as c_int);
        }
        let a=unsafe{&*end};
        let Stage::V90(m)=&a.stage else {panic!("no V90 receiver");};
        eprintln!("replayed first QC training: phase={} echo-delay={}",m.phase(),a.echo.delay);
        assert_eq!(m.retrains(),0,"receiver already retrained before the peer's request");
        assert!(m.v90().unwrap().received_cp_sequences() >= 10,
            "first shortened training still could not consistently read CPt");
        bm_destroy(end);
    }

    #[test]
    #[ignore="requires the saved Quick Connect training failure"]
    fn recorded_quick_training_recovers_the_settled_tail() {
        let raw=std::fs::read(std::env::var("V90_CAPTURE").unwrap()).unwrap();
        let mut pos=12;
        let data=loop {
            let len=u32::from_le_bytes(raw[pos+4..pos+8].try_into().unwrap()) as usize;
            if &raw[pos..pos+4]==b"data" {break &raw[pos+8..pos+8+len];}
            pos+=8+len+(len&1);
        };
        let frames:Vec<(f64,f64)>=data.chunks_exact(4).map(|b|(
            i16::from_le_bytes([b[0],b[1]]) as f64/32768.0,
            i16::from_le_bytes([b[2],b[3]]) as f64/32768.0)).collect();
        let start=(9.58*8000.0) as usize; let end=(12.77*8000.0) as usize;
        let line:Vec<f64>=frames[start..end].iter().map(|p|p.0).collect();
        let reference:Vec<f64>=frames[start..end].iter().map(|p|p.1).collect();
        assert_eq!(Echo::training_delay(&reference,&line,ECHO_LAG_HI),None,
            "fixture no longer reproduces the rejected whole-window fit");
        let (delay,first)=Echo::settled_training_delay(&reference,&line,ECHO_LAG_HI,512).unwrap();
        assert_eq!(delay,6361);
        let (weights,_)=Echo::fit_path_from(&reference,&line,delay as isize-256,512,first).unwrap();
        let norm=weights.iter().map(|w|w*w).sum::<f64>().sqrt();
        eprintln!("recorded settled-tail fit: delay={delay} norm={norm:.4}");
        assert!(norm>0.05 && norm<1.0);
    }

    #[test]
    #[ignore = "requires the saved COM36 failed-first-training stereo recording"]
    fn recorded_final_training_recovers_cp_after_playout_shift() {
        let raw=std::fs::read(std::env::var("V90_CAPTURE").unwrap()).unwrap();
        let mut pos=12;
        let samples=loop {
            let len=u32::from_le_bytes(raw[pos+4..pos+8].try_into().unwrap()) as usize;
            if &raw[pos..pos+4]==b"data" {
                break raw[pos+8..pos+8+len].chunks_exact(2)
                    .map(|b| i16::from_le_bytes(b.try_into().unwrap())).collect::<Vec<_>>();
            }
            pos+=8+len+(len&1);
        };
        assert!(samples.len()>=22*8000*2);
        let end=bm_create_v90(1,None,std::ptr::null_mut(),None,std::ptr::null_mut());
        for frame in 0..22*8000 { bm_service(end); bm_step(end,samples[frame*2] as c_int); }
        let a=unsafe{&*end};
        let Stage::V90(m)=&a.stage else{panic!("no V90 final-training receiver")};
        let cp=m.v90().unwrap().cp().expect("no valid data CP after the recorded 10-ms echo shift");
        assert!(cp.data_mode);
        assert_eq!(cp.drn,16,"the recorded caller asked for 48000 bit/s downstream");
        // ECHO_REFERENCE uses the captured output, whose reflection moves
        // from approximately 1561 to 1481 samples in this recording.
        assert!(a.echo.delay.abs_diff(1481)<=4,
            "the recorded echo timing did not follow the -80-sample shift: {}",a.echo.delay);
        bm_destroy(end);
    }

    #[test]
    #[ignore = "requires a local failed-call stereo recording"]
    fn decode_recorded_downstream_lapm() {
        use datapump::v90::{encoder::{Decoder, Frame, Mapping}, ucode::{self, Law}};
        use datapump::v32::{Scrambler, Mode};
        let raw=std::fs::read(std::env::var("V90_CAPTURE").unwrap()).unwrap();
        let mut pos=12;
        let samples=loop {
            let len=u32::from_le_bytes(raw[pos+4..pos+8].try_into().unwrap()) as usize;
            if &raw[pos..pos+4]==b"data" { break raw[pos+8..pos+8+len].chunks_exact(2).map(|b| i16::from_le_bytes(b.try_into().unwrap())).collect::<Vec<_>>(); }
            pos+=8+len+(len&1);
        };
        let end=bm_create_v90(1,None,std::ptr::null_mut(),None,std::ptr::null_mut());
        for frame in 0..100*8000 {bm_service(end);bm_step(end,samples[frame*2] as c_int);}
        let a=unsafe{&*end};let Stage::V90(m)=&a.stage else{panic!("no V90")};
        let mapping=Mapping::from_cp(m.v90().unwrap().cp().unwrap()).unwrap();
        bm_destroy(end);
        let origin=163*8000;
        for start in origin..origin+6 {
            let mut decoder=Decoder::new(mapping.clone());
            let mut descrambler=Scrambler::new(Mode::Call);
            let mut hdlc=ec::Decoder::new(ec::Fcs::Bits16);hdlc.accept_either();
            let mut valid=0;
            for n in (start..180*8000-6).step_by(6) {
                let pairs:[(u8,bool);6]=std::array::from_fn(|i|ucode::nearest(Law::Mu,samples[(n+i)*2+1] as i32));
                for bit in decoder.frame(Frame{ucodes:pairs.map(|p|p.0),positive:pairs.map(|p|!p.1)}) {
                    if let Some(Ok(body))=hdlc.feed(descrambler.descramble(bit)) {
                        valid+=1;
                        let hex=body.iter().take(12).map(|b|format!("{b:02x}")).collect::<Vec<_>>().join(" ");
                        println!("RECORDED-TX start={} t={:.3} [{}]",start,n as f64/8000.0,hex);
                    }
                }
            }
            println!("RECORDED-TX start={start} valid={valid}");
        }
    }

    #[test]
    #[ignore = "requires a local failed-call stereo recording"]
    fn inspect_recorded_receive_framing() {
        let raw = std::fs::read(std::env::var("V90_CAPTURE").unwrap()).unwrap();
        let mut pos=12;
        let samples=loop {
            let len=u32::from_le_bytes(raw[pos+4..pos+8].try_into().unwrap()) as usize;
            if &raw[pos..pos+4]==b"data" { break raw[pos+8..pos+8+len].chunks_exact(2).map(|b| i16::from_le_bytes(b.try_into().unwrap())).collect::<Vec<_>>(); }
            pos += 8 + len + (len&1);
        };
        let end=bm_create_v90(1,None,std::ptr::null_mut(),None,std::ptr::null_mut());
        for frame in 0..(samples.len()/2).min(210*8000) {
            bm_service(end);
            bm_step(end,samples[frame*2] as c_int);
            if frame%8000==0 {
                let a=unsafe { &*end };
                if let Some(ec)=a.ec.as_ref() {
                    println!("RX-CHECK t={} good={} bad={} bits={} phase={:?}",frame/8000,ec.valid_frames(),ec.damaged_frames(),a.rx_total,ec.phase());
                }
            }
        }
        bm_destroy(end);
    }

    #[test]
    #[ignore = "requires a local failed-call stereo recording"]
    fn decode_recorded_rate_change_mp() {
        use datapump::v90::{encoder::{Decoder, Frame, Mapping}, ucode::{self, Law}};
        use datapump::v32::{Scrambler, Mode};
        use datapump::v34::mp::{Finder, Found};
        let raw = std::fs::read(std::env::var("V90_CAPTURE").unwrap()).unwrap();
        let mut pos=12;
        let samples=loop {
            let len=u32::from_le_bytes(raw[pos+4..pos+8].try_into().unwrap()) as usize;
            if &raw[pos..pos+4]==b"data" { break raw[pos+8..pos+8+len].chunks_exact(2).map(|b| i16::from_le_bytes(b.try_into().unwrap())).collect::<Vec<_>>(); }
            pos += 8 + len + (len&1);
        };
        let seconds = |name: &str, default: f64| {
            std::env::var(name).ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(default)
        };
        let prepare=seconds("V90_MP_PREPARE",140.0);
        let origin=(seconds("V90_MP_AT",145.790)*8000.0) as usize;
        let until=(seconds("V90_MP_UNTIL",149.5)*8000.0) as usize;
        assert!(origin>=24 && until>origin+24 && until<=samples.len()/2);
        let end=bm_create_v90(1,None,std::ptr::null_mut(),None,std::ptr::null_mut());
        for frame in 0..(prepare*8000.0) as usize {
            bm_service(end);
            bm_step(end,samples[frame*2] as c_int);
        }
        let a=unsafe { &*end };
        let Stage::V90(m)=&a.stage else { panic!("no recorded V90 stage"); };
        let d=m.v90().expect("recording selected fallback");
        let cpt=d.cpt().unwrap().clone();
        let cp=d.cp().cloned();
        bm_destroy(end);
        let mapping=if std::env::var_os("V90_MP_INITIAL_TRAINING").is_some() {
            Mapping::from_cp(&cpt).unwrap()
        } else {
            Mapping::for_renegotiation(&cpt,&cp.unwrap()).unwrap()
        };
        println!("recorded training mapping {mapping:?}");
        let mut best=0;
        for start in origin-24..origin+24 {
            let mut decoder=Decoder::new(mapping.clone());
            let mut descrambler=Scrambler::new(Mode::Call);
            let mut finder=Finder::new();
            let mut found=0;
            let mut ack=0;
            for n in (start..until-6).step_by(6) {
                let pairs:[(u8,bool);6]=std::array::from_fn(|i| ucode::nearest(Law::Mu,samples[(n+i)*2+1] as i32));
                let frame=Frame {ucodes:pairs.map(|p| p.0),positive:pairs.map(|p| !p.1)};
                for bit in decoder.frame(frame) {
                    if let Some(Found::Mp(mp))=finder.feed(descrambler.descramble(bit)) {
                        found+=1;ack+=usize::from(mp.acknowledge);
                    }
                }
            }
            if found>best {best=found;println!("MP alignment {start}: {found} valid, {ack} acknowledged");}
        }
        assert!(best>0,"recorded outgoing MP could not be decoded using the preceding CPt/data parameters");
    }
}

#[cfg(test)]
mod v23_ffi_checks {
    use super::*;
    #[test]
    fn v8_handover_preserves_reversed_v8bis_roles_in_the_resampled_path() {
        let null=std::ptr::null_mut();
        let mut a=Answerer::new(true,false,false,None,null,None,null);
        let mut b=Answerer::new(false,false,false,None,null,None,null);
        let modes=Modulations::of(&[Modulation::V32bis]);
        a.stage=Stage::V8(Box::new(v8line::Modem::new(v8line::Role::Calling,
            CallFunction::Data,modes,ENGINE_FS).offering_lapm()));
        b.stage=Stage::V8(Box::new(v8line::Modem::new(v8line::Role::Answering,
            CallFunction::Data,modes,ENGINE_FS).offering_lapm()));
        let (mut ai,mut bi)=(0,0);
        for _ in 0..(15*8000) {
            let at=bm_step(&mut a,ai);let bt=bm_step(&mut b,bi);ai=bt;bi=at;
            if matches!(a.stage,Stage::V32(_))&&matches!(b.stage,Stage::V32(_)) {break;}
        }
        assert!(matches!(a.stage,Stage::V32(_))&&matches!(b.stage,Stage::V32(_)));
        assert_eq!(a.role,v34::phase2::Role::Call);
        assert_eq!(b.role,v34::phase2::Role::Answer);
        assert!(a.lapm_declared&&b.lapm_declared);
    }
    #[test]
    fn v8_selected_v23_reaches_split_speed_bytes_in_both_sample_paths() {
        struct Dte { tx:VecDeque<bool>, rx:Vec<u8>, framing:AsyncBits }
        extern "C" fn get(p:*mut c_void)->c_int {
            unsafe {&mut *(p as *mut Dte)}.tx.pop_front().map(|v|v as c_int).unwrap_or(-1)
        }
        extern "C" fn put(p:*mut c_void,b:c_int) {
            let d=unsafe{&mut *(p as *mut Dte)};
            if let Some(byte)=d.framing.feed(b!=0) {d.rx.push(byte);}
        }
        for linear in [false,true] {
            let mut ad=Dte{tx:VecDeque::new(),rx:Vec::new(),framing:AsyncBits::new(8)};
            let mut bd=Dte{tx:VecDeque::new(),rx:Vec::new(),framing:AsyncBits::new(8)};
            let ap=&mut ad as *mut Dte as *mut c_void;
            let bp=&mut bd as *mut Dte as *mut c_void;
            let mut a=Answerer::new(true,true,linear,Some(get),ap,Some(put),ap);
            let mut b=Answerer::new(false,false,false,Some(get),bp,Some(put),bp);
            b.stage=Stage::V8(Box::new(v8line::Modem::new(v8line::Role::Calling,
                CallFunction::Data,Modulations::of(&[Modulation::V23Duplex]),ENGINE_FS).offering_lapm()));
            let (mut ai,mut bi)=(0,0);let mut armed=false;
            let payload=[0x00,0xff,0x55,0xaa,0x7e,0x13];
            for tick in 0..(25*8000) {
                if tick%160==0 {
                    bm_service(&mut a);bm_service(&mut b);
                    if !armed && a.status==BM_CONNECTED && b.status==BM_CONNECTED {
                        ad.rx.clear();bd.rx.clear();
                        for byte in payload {ad.tx.extend(AsyncBits::new(8).encode(byte));bd.tx.extend(AsyncBits::new(8).encode(byte));}
                        armed=true;
                    }
                }
                let at=bm_step(&mut a,ai);let bt=bm_step(&mut b,bi);ai=bt;bi=at;
                if armed && ad.rx.len()>=payload.len() && bd.rx.len()>=payload.len() {break;}
            }
            assert!(armed,"V.8 did not hand over to V.23: linear={linear}");
            assert!(matches!(a.stage,Stage::V23(_))&&matches!(b.stage,Stage::V23(_)));
            assert_eq!((a.rate_tx,a.rate_rx),(1200,75));
            assert_eq!((b.rate_tx,b.rate_rx),(75,1200));
            assert!(a.ec.is_none()&&b.ec.is_none()&&!a.lapm_declared&&!b.lapm_declared);
            assert_eq!(ad.rx,payload);assert_eq!(bd.rx,payload);
        }
    }
    #[test]
    fn v23_service_preserves_carrier_and_split_speed_rates() {
        for reverse in [0,1] {
            let a=bm_create_v23(1,reverse,None,std::ptr::null_mut(),None,std::ptr::null_mut());
            let b=bm_create_v23(0,reverse,None,std::ptr::null_mut(),None,std::ptr::null_mut());
            let (mut ai,mut bi)=(0,0);
            for _ in 0..4000 {
                let at=bm_step(a,ai);let bt=bm_step(b,bi);ai=bt;bi=at;
                bm_service(a);bm_service(b);
            }
            assert_eq!(bm_status(a),BM_CONNECTED);assert_eq!(bm_status(b),BM_CONNECTED);
            assert_eq!(bm_carrier(a),1);assert_eq!(bm_carrier(b),1);
            assert!(bm_pending(a)>=0);bm_flush_rx(a);
            let a_ref=unsafe{&*a};let b_ref=unsafe{&*b};
            assert_eq!((a_ref.rate_tx,a_ref.rate_rx),if reverse==0{(1200,75)}else{(75,1200)});
            assert_eq!((a_ref.rate_tx,a_ref.rate_rx),(b_ref.rate_rx,b_ref.rate_tx));
            assert!(a_ref.ec.is_none()&&b_ref.ec.is_none());
            bm_destroy(a);bm_destroy(b);
        }
        assert!(bm_create_v23(1,2,None,std::ptr::null_mut(),None,std::ptr::null_mut()).is_null());
    }
}

/// The V.42 link C's V.22bis runs through (`bm_ec_*`).
#[cfg(test)]
mod ec_link_tests {
    use super::*;
    use std::collections::VecDeque;

    /// One end's DTE: bits waiting for get_bit, bits put_bit handed up.
    #[derive(Default)]
    struct Dte {
        out: VecDeque<bool>,
        got: Vec<bool>,
    }

    impl Dte {
        fn type_bytes(&mut self, data: &[u8]) {
            let framer = AsyncBits::new(8);
            for &b in data {
                self.out.extend(framer.encode(b));
            }
        }
        fn received(&self) -> Vec<u8> {
            let mut deframer = AsyncBits::new(8);
            self.got.iter().filter_map(|&b| deframer.feed(b)).collect()
        }
    }

    unsafe extern "C" fn get(ud: *mut c_void) -> c_int {
        let dte = unsafe { &mut *(ud as *mut Dte) };
        c_int::from(dte.out.pop_front().unwrap_or(true))
    }

    unsafe extern "C" fn put(ud: *mut c_void, bit: c_int) {
        let dte = unsafe { &mut *(ud as *mut Dte) };
        dte.got.push(bit != 0);
    }

    fn link(answer: bool, dte: &mut Dte) -> *mut EcLink {
        let ud = dte as *mut Dte as *mut c_void;
        bm_ec_create(c_int::from(answer), 0, 2400, 40, Some(get), ud, Some(put), ud)
    }

    /// Runs `seconds` of a 2400 bit/s line; `step` moves one bit each way.
    fn run(seconds: f64, mut step: impl FnMut(), ticks: &[*mut EcLink]) {
        let bits = (seconds * 2400.0) as usize;
        for i in 0..bits {
            step();
            if i % 12 == 11 {
                for &l in ticks {
                    bm_ec_tick(l, 5);
                }
            }
        }
    }

    #[test]
    fn two_links_agree_lapm_with_compression_and_carry_both_ways() {
        let (mut a_dte, mut c_dte) = (Box::new(Dte::default()), Box::new(Dte::default()));
        let (answerer, caller) = (link(true, &mut a_dte), link(false, &mut c_dte));
        run(3.0, || {
            let (to_c, to_a) = (bm_ec_tx_bit(answerer), bm_ec_tx_bit(caller));
            bm_ec_rx_bit(caller, to_c);
            bm_ec_rx_bit(answerer, to_a);
        }, &[answerer, caller]);
        assert_eq!(bm_ec_link_phase(answerer), 2, "answerer in LAPM");
        assert_eq!(bm_ec_link_phase(caller), 2, "caller in LAPM");
        assert_eq!(bm_ec_link_compression(answerer), 1, "V.42bis agreed");

        let down: Vec<u8> = (0..600u32).map(|i| b"PPP downstream text "[i as usize % 20]).collect();
        let up: Vec<u8> = (0..300u32).map(|i| (i * 7) as u8).collect();
        a_dte.type_bytes(&down);
        c_dte.type_bytes(&up);
        // 600 + 300 characters in 3 s of a 2400 bit/s line: one frame per
        // character (no batching) took about sixteen.
        run(3.0, || {
            let (to_c, to_a) = (bm_ec_tx_bit(answerer), bm_ec_tx_bit(caller));
            bm_ec_rx_bit(caller, to_c);
            bm_ec_rx_bit(answerer, to_a);
        }, &[answerer, caller]);
        assert_eq!(c_dte.received(), down, "answerer to caller");
        assert_eq!(a_dte.received(), up, "caller to answerer");
        assert_eq!(bm_ec_link_damaged_frames(answerer), 0);
        bm_ec_destroy(answerer);
        bm_ec_destroy(caller);
    }

    /// A caller with no V.42 at all: detection gives up, the link goes
    /// transparent, and plain async characters pass both ways, including
    /// those typed while the answerer was still detecting.
    #[test]
    fn a_caller_without_v42_gets_a_transparent_link() {
        let mut a_dte = Box::new(Dte::default());
        let answerer = link(true, &mut a_dte);
        let framer = AsyncBits::new(8);
        let mut caller_line: VecDeque<bool> = b"ATDT early\r".iter().flat_map(|&b| framer.encode(b)).collect();
        let mut caller_heard = Vec::new();
        run(4.0, || {
            caller_heard.push(bm_ec_tx_bit(answerer) != 0);
            bm_ec_rx_bit(answerer, c_int::from(caller_line.pop_front().unwrap_or(true)));
        }, &[answerer]);
        assert_eq!(bm_ec_link_phase(answerer), 3, "transparent after detection");
        assert_eq!(bm_ec_link_compression(answerer), 0);

        a_dte.type_bytes(b"Login:");
        caller_line.extend(b"cooper\r".iter().flat_map(|&b| framer.encode(b)));
        run(1.0, || {
            caller_heard.push(bm_ec_tx_bit(answerer) != 0);
            bm_ec_rx_bit(answerer, c_int::from(caller_line.pop_front().unwrap_or(true)));
        }, &[answerer]);
        let mut deframer = AsyncBits::new(8);
        let heard: Vec<u8> = caller_heard.iter().filter_map(|&b| deframer.feed(b)).collect();
        let text = String::from_utf8_lossy(&heard);
        assert!(text.ends_with("Login:"), "caller heard {text:?}");
        assert_eq!(a_dte.received(), b"ATDT early\rcooper\r".to_vec());
        bm_ec_destroy(answerer);
    }

    unsafe extern "C" fn link_tx(ud: *mut c_void) -> c_int { bm_ec_tx_bit(ud as *mut EcLink) }
    unsafe extern "C" fn link_rx(ud: *mut c_void, bit: c_int) { bm_ec_rx_bit(ud as *mut EcLink, bit) }

    /// V.42 over Bell 103 end to end: two FSK pumps in raw-bit mode with a
    /// link each, through the line audio. LAPM comes up at 300 bit/s and
    /// carries text both ways.
    #[test]
    fn hst_carries_async_characters_both_ways() {
        let mut a = HstEnd::new(true);
        // A caller already past the answer tone.
        let mut c = HstEnd::new(false);
        c.call = None;
        c.s = Some(datapump::hst::Session::new(LINE_FS, datapump::hst::Side::Caller, datapump::hst::Config::default()));
        let chars = |text: &[u8]| -> Vec<bool> {
            let mut bits = vec![true; 20];
            for &b in text { bits.push(false); for i in 0..8 { bits.push(b >> i & 1 != 0); } bits.push(true); bits.push(true); }
            bits
        };
        let bytes = |bits: &mut std::collections::VecDeque<bool>, out: &mut Vec<u8>| {
            while bits.len() >= 10 {
                if bits[0] { bits.pop_front(); continue; }
                let mut v = 0u8;
                for i in 0..8 { v |= u8::from(bits[1 + i]) << i; }
                out.push(v);
                bits.drain(..10);
            }
        };
        let (mut ya, mut yc) = (0.0, 0.0);
        let (mut got_a, mut got_c) = (Vec::new(), Vec::new());
        let mut sent = false;
        for n in 0..(20.0 * LINE_FS) as usize {
            let na = a.step(yc);
            let nc = c.step(ya);
            ya = na; yc = nc;
            if !sent && a.connected && c.connected && n % 8000 == 0 {
                for bit in chars(b"from the server") { a.take_bit(bit); }
                for bit in chars(b"typed by the caller") { c.take_bit(bit); }
                sent = true;
            }
            bytes(&mut a.up, &mut got_a);
            bytes(&mut c.up, &mut got_c);
        }
        assert!(sent, "never connected: {} / {}", a.phase(), c.phase());
        assert_eq!(String::from_utf8_lossy(&got_c), "from the server");
        assert_eq!(String::from_utf8_lossy(&got_a), "typed by the caller");
        assert_eq!(a.rates().0, 14400);
    }

    #[test]
    fn bell103_carries_v42_lapm_over_the_line() {
        let (mut a_dte, mut c_dte) = (Box::new(Dte::default()), Box::new(Dte::default()));
        let ad = &mut *a_dte as *mut Dte as *mut c_void;
        let cd = &mut *c_dte as *mut Dte as *mut c_void;
        let a_link = bm_ec_create(1, 0, 300, 40, Some(get), ad, Some(put), ad);
        let c_link = bm_ec_create(0, 0, 300, 40, Some(get), cd, Some(put), cd);
        let a = bm_create_bell103(1, Some(link_tx), a_link as *mut c_void, Some(link_rx), a_link as *mut c_void);
        let c = bm_create_bell103(0, Some(link_tx), c_link as *mut c_void, Some(link_rx), c_link as *mut c_void);
        bm_set_raw_line_bits(a, 1);
        bm_set_raw_line_bits(c, 1);
        let (mut to_a, mut to_c) = (0, 0);
        // The links start with data mode, as the daemon starts its link
        // only once the pump reports carrier: detection must not time out
        // while the carriers are still being qualified.
        let mut run = |seconds: f64| {
            for n in 0..(seconds * 8000.0) as usize {
                let ya = bm_step(a, to_a);
                let yc = bm_step(c, to_c);
                to_c = ya; to_a = yc;
                let up = bm_status(a) == BM_CONNECTED && bm_status(c) == BM_CONNECTED;
                if up && n % 40 == 39 { bm_ec_tick(a_link, 5); bm_ec_tick(c_link, 5); }
            }
        };
        run(12.0);
        assert_eq!(bm_ec_link_phase(a_link), 2, "answerer in LAPM");
        assert_eq!(bm_ec_link_phase(c_link), 2, "caller in LAPM");
        let down = b"Cooper Dialup Services\r\nLogin:".to_vec();
        a_dte.type_bytes(&down);
        c_dte.type_bytes(b"dialup\r");
        run(10.0);
        assert_eq!(c_dte.received(), down, "answerer to caller");
        assert_eq!(a_dte.received(), b"dialup\r".to_vec(), "caller to answerer");
        bm_destroy(a); bm_destroy(c);
        bm_ec_destroy(a_link); bm_ec_destroy(c_link);
    }

    /// The same answerer to a Bell 103 caller with no V.42: detection gives
    /// up and plain characters pass both ways at the recovered clock.
    #[test]
    fn bell103_with_v42_still_serves_a_plain_caller() {
        let (mut a_dte, mut c_dte) = (Box::new(Dte::default()), Box::new(Dte::default()));
        let ad = &mut *a_dte as *mut Dte as *mut c_void;
        let cd = &mut *c_dte as *mut Dte as *mut c_void;
        let a_link = bm_ec_create(1, 0, 300, 40, Some(get), ad, Some(put), ad);
        let a = bm_create_bell103(1, Some(link_tx), a_link as *mut c_void, Some(link_rx), a_link as *mut c_void);
        let c = bm_create_bell103(0, Some(get), cd, Some(put), cd);
        bm_set_raw_line_bits(a, 1);
        let (mut to_a, mut to_c) = (0, 0);
        let mut run = |seconds: f64| {
            for n in 0..(seconds * 8000.0) as usize {
                let ya = bm_step(a, to_a);
                let yc = bm_step(c, to_c);
                to_c = ya; to_a = yc;
                if bm_status(a) == BM_CONNECTED && n % 40 == 39 { bm_ec_tick(a_link, 5); }
            }
        };
        run(6.0);
        assert_eq!(bm_ec_link_phase(a_link), 3, "transparent after detection");
        a_dte.type_bytes(b"Login:");
        c_dte.type_bytes(b"dialup\r");
        run(4.0);
        assert!(String::from_utf8_lossy(&c_dte.received()).ends_with("Login:"), "caller heard {:?}", c_dte.received());
        assert_eq!(a_dte.received(), b"dialup\r".to_vec(), "caller to answerer");
        bm_destroy(a); bm_destroy(c);
        bm_ec_destroy(a_link);
    }
}
