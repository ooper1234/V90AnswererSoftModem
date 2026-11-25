//! The LAPM state machine (V.42 8.3 to 8.9).
//!
//! Establishment, release, and numbered information transfer with go-back-N
//! recovery. Time is supplied by the caller through [`Lapm::tick`] rather than
//! read from a clock, so the whole protocol is testable at whatever speed a
//! test wants.
//!
//! Not yet covered: break transfer (V.42 8.13), the multi-frame form of
//! selective reject (8.4.5.2) and the optional T402/T403 timers. XID is
//! covered, but not here: what it negotiates spans all three layers --
//! compression above, frame check sequence below -- so it is driven by the
//! stack that owns them.

use std::collections::{BTreeMap, VecDeque};

use crate::frame::{Frame, Kind, MODULUS, Role};

/// Maximum octets in an information field. V.42 9.2.3 sets the default at 128
/// and allows XID to negotiate it per direction.
pub const DEFAULT_N401: usize = 128;

/// Outstanding I frames allowed. V.42 9.2.4 sets the default at 15.
pub const DEFAULT_K: u8 = 15;

/// Acknowledgement timer in milliseconds, when the rate is not known.
///
/// V.42 9.2.1 defines what T401 is for but gives no default value at all, only
/// pointing at Appendix IV for the factors involved. Three seconds is
/// comfortable at 300 bps, where a full 128-octet frame alone takes over four
/// seconds to send, and is the sort of value real modems used at low speed.
///
/// It is far too long at any rate this modem actually runs V.42 at, which is
/// why [`t401_for`] exists.
pub const DEFAULT_T401_MS: u32 = 3000;

/// The acknowledgement timer for a given line rate (V.42 Appendix IV).
///
/// The appendix does not give a value, it gives a sum: T401 must be at least
/// "Ta + Tb + Tc + Td + Te + Tf" -- the propagation each way, the processing at
/// each end, the time to finish whatever frame was already going out, and the
/// time to send the acknowledgement. Only two of those are large, and both are
/// the line rate: a full information frame in progress, and the supervisory
/// frame that answers it.
///
/// Three seconds regardless of rate is what this was, and it is nearly seven
/// times the transmission terms at 2400 bit/s. That does not sound like much
/// until the retransmission limit multiplies it: a far end that never answers
/// a SABME costs three seconds an attempt, and on a real call that meant nine
/// seconds during which the terminal had been told nothing at all, on a
/// connection that was up and working at 2400.
///
/// The propagation allowance is measured rather than guessed at. Half a second
/// for the pair of them was the guess, and a call to a board over a SIP trunk
/// says otherwise: SABME out at 16.590 s, UA back at 17.870 s, and 466 ms of
/// that is the transmission terms at 2400 bit/s. That leaves 814 ms of
/// propagation and processing on an ordinary call, so a 500 ms allowance
/// guarantees a duplicate SABME on every one of them -- and a duplicate SABME
/// is not merely wasteful, since a far end that honours it resets its sequence
/// variables underneath a link that was working.
///
/// A second covers that line with room to spare. A slower one is covered by
/// [`Lapm::tick`] raising the timer as it learns, and better still by
/// [`t401_for_line`] where the line has been measured, because no constant can
/// be right for every path.
pub fn t401_for(bits_per_second: u32) -> u32 {
    PROPAGATION_MS + transmission_ms(bits_per_second)
}

/// The same, on a line whose round trip the data pump has already measured.
///
/// Measured is the right word for Ta + Tb, and the start-up measures it before
/// error control ever begins: V.34 in phase 2, V.32 in its counter/timer. The
/// allowance in [`t401_for`] is a guess at them and was too small by the time
/// it met a line that V.34 put at 1125 ms. At 28 800 bit/s the transmission
/// terms are 38 ms, so T401 came to 1.04 s, and the far end took 1.19 s to
/// answer a SABME -- which was therefore sent twice, on every call over that
/// line, and a far end that honoured the second reset its sequence numbers
/// under a link that was already carrying its banner.
///
/// Half as much again as the measurement, which is the margin [`Lapm`] itself
/// uses when it learns from an answer: Te and Tf, the processing at each end,
/// are not in it, and neither is the far end's queue. Never less than the
/// unmeasured allowance, since a short line with a slow far end still needs
/// that much, and never more than the timer is ever allowed to grow to.
pub fn t401_for_line(bits_per_second: u32, round_trip_ms: u32) -> u32 {
    let measured = round_trip_ms.saturating_add(round_trip_ms / 2);
    let t401 = PROPAGATION_MS.max(measured) + transmission_ms(bits_per_second);
    t401.min(MAX_T401_MS)
}

/// Ta + Tb + Te + Tf where nothing has measured them: the propagation each way
/// and the processing at each end, which is the part that does not depend on
/// the line rate.
pub const PROPAGATION_MS: u32 = 1000;

/// Tc + Td: the longest frame that could already be going out -- information
/// field plus address, control and check sequence -- and the supervisory frame
/// that acknowledges it.
fn transmission_ms(bits_per_second: u32) -> u32 {
    const FRAME_BITS: u32 = (DEFAULT_N401 as u32 + 6) * 8;
    const ACK_BITS: u32 = 6 * 8;
    (FRAME_BITS + ACK_BITS) * 1000 / bits_per_second.max(1)
}

/// Retransmission limit.
///
/// V.42 9.2.2 specifies no default, requiring only a minimum of 1. Appendix
/// III.2 recommends "a relatively large value (e.g. 16)", and says why: by the
/// time establishment is being attempted the detection phase has already
/// finished, so each end knows the other does LAPM, and giving up early on a
/// modem known to be there throws away a connection that noise alone was
/// spoiling. The appendix's own caveat -- keep it small if the detection phase
/// was omitted, to fall back quickly to a modem that does not do this at all
/// -- does not apply here, because this modem never omits it.
pub const DEFAULT_N400: u32 = 16;

/// Retransmissions to allow when the detection phase did not confirm anything.
///
/// The other half of Appendix III.2: a large N400 is right when detection has
/// established that the far end does LAPM, and wrong when it has not, because
/// then every retry is time spent talking at a modem in a language it may not
/// speak. The appendix asks for a small value where the detection phase is
/// omitted, "to accommodate fallback operation with non-error-correcting DCEs
/// and with DCEs which support only the alternative protocol". A far end that
/// named LAPM in V.8 is good evidence and not that confirmation.
pub const UNCONFIRMED_N400: u32 = 3;

/// Tunable protocol parameters (V.42 clause 9.2).
#[derive(Debug, Clone, Copy)]
pub struct Params {
    pub n401: usize,
    pub k: u8,
    pub t401_ms: u32,
    pub n400: u32,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            n401: DEFAULT_N401,
            k: DEFAULT_K,
            t401_ms: DEFAULT_T401_MS,
            n400: DEFAULT_N400,
        }
    }
}

/// Connection state (V.42 8.3, 8.7, 8.8).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Disconnected,
    /// SABME sent, waiting for UA or DM.
    AwaitingEstablishment,
    /// DISC sent, waiting for UA or DM.
    AwaitingRelease,
    Connected,
}

/// Why the connection ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cause {
    /// This end asked to disconnect.
    Local,
    /// The peer sent DISC.
    Peer,
    /// N400 attempts produced no response (V.42 8.3.2.2, 8.7.3).
    NoResponse,
    /// The peer refused or is disconnected.
    Refused,
}

/// What the state machine reports upward.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// The error-corrected connection is up.
    Connected,
    /// The connection ended.
    Released(Cause),
    /// Data recovered from the peer, in order.
    Data(Vec<u8>),
    /// The peer re-established mid-connection, so unacknowledged data was lost
    /// (V.42 8.2.4.3: previously-transmitted I frames are discarded).
    Reset,
    /// The SABME behind the last `Reset` was a late duplicate: the peer had
    /// not re-established, and its acknowledgements still counted from before.
    /// The old sequence state is back and nothing was lost, but anything above
    /// that restarted on the `Reset` (V.42bis dictionaries) is now out of step.
    ResetUndone,
}

/// What a SABME received while connected reset, kept until the peer shows
/// whether it re-established too.
#[derive(Debug, Clone)]
struct PreReset {
    vs: u8,
    va: u8,
    vr: u8,
    unacked: VecDeque<(u8, Vec<u8>)>,
    held: BTreeMap<u8, Vec<u8>>,
}

/// One LAPM entity.
#[derive(Debug)]
pub struct Lapm {
    role: Role,
    dlci: u8,
    params: Params,
    state: State,

    /// Send state variable (V.42 8.2.3.2.2).
    vs: u8,
    /// Acknowledge state variable (V.42 8.2.3.2.3).
    va: u8,
    /// Receive state variable (V.42 8.2.3.2.5).
    vr: u8,

    /// Data from the local DTE waiting to go out.
    pending: VecDeque<Vec<u8>>,
    /// I frames sent but not yet acknowledged, oldest first.
    unacked: VecDeque<(u8, Vec<u8>)>,
    /// Frames queued for transmission.
    out: VecDeque<(Frame, Kind)>,
    /// Events for the control function.
    events: VecDeque<Event>,

    /// The peer told us to stop sending (RNR).
    peer_busy: bool,
    /// A REJ has been sent and not yet resolved, so do not send another
    /// (V.42 8.4.4: only one reject exception at a time).
    reject_sent: bool,
    /// Whether the single-selective-reject procedure was agreed (8.4.5.1).
    srej: bool,
    /// The most I frames the far end may have outstanding towards this end,
    /// as the XID exchange settled it (V.42 9.2.4): what an out-of-sequence
    /// frame is held against. Its own, not [`Params::k`], which is this end's
    /// sending window.
    receive_k: u8,
    /// I frames that arrived ahead of V(R), held until the gap closes.
    ///
    /// The whole of what selective reject buys. Go-back-N throws these away
    /// and asks for all of them again; 8.2.4.8.1 says "I frames that may have
    /// been transmitted following the I frame indicated by the SREJ frame
    /// shall not be retransmitted", which is only possible if the receiver
    /// kept them.
    held: BTreeMap<u8, Vec<u8>>,
    /// An in-sequence I frame arrived and has not yet been acknowledged.
    ack_pending: bool,
    /// The timer-recovery condition (V.42 8.5.3): T401 expired with frames
    /// outstanding, so the peer has been polled and its reply decides what
    /// needs sending again.
    timer_recovery: bool,

    timer: Option<u32>,
    retries: u32,
    /// Time since the oldest command frame that is still unacknowledged.
    ///
    /// Not the same as the timer, and deliberately not reset when the timer
    /// restarts: what is wanted is how long the acknowledgement really took,
    /// across however many attempts it took to get one.
    awaiting_ms: Option<u32>,
    /// T401 expired at least once during the wait `awaiting_ms` measures, so
    /// that wait is the round trip plus however long the frames were lost,
    /// not the round trip. A poll's answer resets `retries` before the data
    /// is acknowledged, so the count cannot say this on its own: 2026-11-17,
    /// a 20 s outage was taken as a 20 s round trip and T401 set to its
    /// ceiling for the rest of the call.
    timed_out: bool,
    /// T401 expiries since an answer that came within the timer, which set
    /// how much longer than the line's value the next wait is. Kept apart
    /// from `retries`, which a re-establishment and a poll's answer reset,
    /// because only a clean answer says the line's value is enough again
    /// (Karn's rule): a far end further away than the timer allows for
    /// answers every attempt late, and a link re-established in a long
    /// outage would otherwise spend its attempts at the shortest wait.
    backoff: u32,
    /// Sequence state from before a SABME that arrived while connected.
    ///
    /// A SABME the peer retransmitted because our UA was slow to reach it can
    /// arrive after the connection is up and carrying data. Its peer is
    /// connected and has not reset; resetting on it anyway leaves the two ends
    /// numbering frames differently, and every frame after is discarded as a
    /// duplicate. 8.2.4.3 is followed -- the reset happens -- but if the
    /// peer's next N(R) or N(S) only makes sense against the old numbering,
    /// the old numbering is put back.
    pre_reset: Option<PreReset>,
    /// Whether, since the link was last established, the peer has
    /// acknowledged a frame or sent one in sequence. Until it has, a SABME on
    /// the connected link is the peer still opening it (see `on_sabme`).
    exchanged: bool,
    /// Time since the peer last showed that it hears us: an acknowledgement
    /// that moved V(A), a response with F set (the answer to a poll of ours),
    /// a reject, or a UA.
    ///
    /// Its own polls and its own data say nothing about that. A real far end
    /// that had stopped reading our signal after a rate renegotiation went
    /// on polling every 0.7 s for over 40 s; with nothing of ours outstanding
    /// no timer was running here, so nothing noticed.
    unheard_ms: u32,
    /// Time since this end last sent a probe (see [`PROBE_AFTER_MS`]).
    since_probe_ms: u32,
    /// Milliseconds since the link was made, for the trace.
    clock_ms: u64,
    /// A diagnostic record of frames handed out and N(R)s heard, while set.
    pub trace: Option<Vec<String>>,
}

/// How long a connected link may go without evidence the peer hears us before
/// it is asked, with an RR command and P set, which it must answer (8.4.2.1).
///
/// Long enough that a link carrying our data never needs one, short enough
/// that a deaf peer shows within a few seconds.
const PROBE_AFTER_MS: u32 = 3000;

/// The most T401 will be allowed to grow to.
///
/// Nothing in V.42 gives a ceiling. This one is the point past which a line is
/// not slow but gone, and it keeps N400 attempts from adding up to minutes.
const MAX_T401_MS: u32 = 6000;

impl Lapm {
    pub fn new(role: Role, dlci: u8, params: Params) -> Self {
        Self {
            role,
            dlci,
            params,
            state: State::Disconnected,
            vs: 0,
            va: 0,
            vr: 0,
            pending: VecDeque::new(),
            unacked: VecDeque::new(),
            out: VecDeque::new(),
            events: VecDeque::new(),
            peer_busy: false,
            reject_sent: false,
            srej: false,
            receive_k: DEFAULT_K,
            held: BTreeMap::new(),
            ack_pending: false,
            timer_recovery: false,
            timer: None,
            retries: 0,
            awaiting_ms: None,
            timed_out: false,
            backoff: 0,
            unheard_ms: 0,
            since_probe_ms: 0,
            clock_ms: 0,
            trace: None,
            pre_reset: None,
            exchanged: false,
        }
    }

    /// Change the retransmission limit before anything has been established.
    ///
    /// How much patience is warranted is not known when the entity is built:
    /// it depends on how the detection phase came out, which happens later.
    ///
    /// This is the limit on the procedures this entity runs, and not the one
    /// the XID exchange above it uses: 9.2.2 lets "the two error-correcting
    /// entities associated with an error-corrected connection ... operate with
    /// a different value of N400", and by the same token the two questions are
    /// separate. Establishment is retried at a far end the detection phase has
    /// shown does LAPM; the XID exchange is asking a far end that has shown
    /// nothing of the sort whether it negotiates at all.
    pub fn set_retransmissions(&mut self, n400: u32) {
        self.params.n400 = n400;
    }

    /// Use the selective retransmission procedure (V.42 8.4.5.1).
    ///
    /// Optional, and only after both ends have said so in XID: 8.4.5.1 has an
    /// end that did not agree treat an SREJ as an unrecognized control field,
    /// which under 8.5.5 ends the connection.
    /// The longest information field and the most outstanding I frames
    /// this end sends, as the XID exchange settled them (V.42 9.2.3, 9.2.4).
    pub fn set_send_limits(&mut self, n401: usize, k: u8) {
        self.params.n401 = n401.max(1);
        self.params.k = k.clamp(1, MODULUS - 1);
    }

    /// The far end's window towards this end, as the XID exchange settled it.
    pub fn set_receive_window(&mut self, k: u8) {
        self.receive_k = k.clamp(1, MODULUS - 1);
    }

    /// What [`Self::set_send_limits`] last set, or the defaults.
    pub fn send_limits(&self) -> (usize, u8) {
        (self.params.n401, self.params.k)
    }

    pub fn set_selective_reject(&mut self, on: bool) {
        self.srej = on;
    }

    /// Whether selective retransmission is in use.
    pub fn selective_reject(&self) -> bool {
        self.srej
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn is_connected(&self) -> bool {
        self.state == State::Connected
    }

    /// Age of the oldest unacknowledged I frame, excluding peer flow control.
    /// Retransmissions and duplicate acknowledgements do not mean progress.
    pub fn stalled_transmit_ms(&self) -> Option<u32> {
        (self.is_connected() && self.va != self.vs && !self.peer_busy)
            .then_some(self.awaiting_ms).flatten()
    }


    /// How long, while connected, since the peer last showed it hears us.
    ///
    /// An idle link is probed every [`PROBE_AFTER_MS`], so a peer that hears
    /// us keeps this below about that plus a round trip.
    pub fn unheard_ms(&self) -> Option<u32> {
        self.is_connected().then_some(self.unheard_ms)
    }

    fn heard(&mut self) {
        self.unheard_ms = 0;
    }

    /// Poll the peer now, as the idle probe does: an RR command with P set,
    /// which it must answer (8.4.2.1). Nothing while not connected.
    pub fn probe(&mut self) {
        if self.state == State::Connected {
            self.send(Frame::Rr { nr: self.vr, pf: true }, Kind::Command);
            self.ack_pending = false;
            self.since_probe_ms = 0;
        }
    }

    /// Frames to hand to the HDLC layer.
    pub fn poll_transmit(&mut self) -> Option<(Frame, Kind)> {
        self.pump();
        let next = self.out.pop_front();
        if let (Some(t), Some((f, _))) = (self.trace.as_mut(), next.as_ref()) {
            let what = match f {
                Frame::I { ns, nr, poll, info } => format!("I ns {ns} nr {nr} p {poll} len {}", info.len()),
                Frame::Rr { nr, pf } => format!("RR nr {nr} pf {pf}"),
                other => format!("{other:?}").chars().take(40).collect(),
            };
            t.push(format!("{} out {what} va {} vs {}", self.clock_ms, self.va, self.vs));
        }
        next
    }

    /// Events for the control function.
    pub fn poll_event(&mut self) -> Option<Event> {
        self.events.pop_front()
    }

    /// Ask for an error-corrected connection (V.42 8.3).
    ///
    /// Calling this while already connected re-establishes, which V.42 8.6
    /// provides for: the sequence variables are reset at both ends and
    /// unacknowledged frames are discarded. Only an establishment already in
    /// progress is ignored, since a second SABME would achieve nothing.
    pub fn connect(&mut self) {
        if self.state == State::AwaitingEstablishment {
            return;
        }
        self.reset_variables();
        self.state = State::AwaitingEstablishment;
        self.retries = 0;
        // The P bit is set so the peer's UA can be matched to this request.
        self.send(Frame::Sabme { poll: true }, Kind::Command);
        self.start_timer();
    }

    /// Ask to release the connection (V.42 8.7.2).
    pub fn disconnect(&mut self) {
        if self.state == State::Disconnected {
            return;
        }
        self.pending.clear();
        self.unacked.clear();
        self.state = State::AwaitingRelease;
        self.retries = 0;
        // V.42 8.7.2 note: DISC always carries P set to 1, so a DM response
        // cannot be misread as unsolicited.
        self.send(Frame::Disc { poll: true }, Kind::Command);
        self.start_timer();
    }

    /// Bytes accepted from above and not yet put in a frame.
    ///
    /// What anyone upstream needs to know before handing over more. There is
    /// no back pressure in `send_data` -- it takes whatever it is given -- so
    /// the only thing stopping a caller filling memory with a file is a caller
    /// that asks first.
    pub fn queued(&self) -> usize {
        self.pending.iter().map(Vec::len).sum()
    }

    /// Nothing waiting to be framed and nothing sent unacknowledged: the far
    /// end has every octet handed down so far.
    pub fn all_acknowledged(&self) -> bool {
        self.pending.is_empty() && self.unacked.is_empty()
    }

    /// Compression reinitialization invalidates already encoded octets that
    /// have not reached the serializer, just as it invalidates unacked data.
    pub fn discard_pending(&mut self) {
        self.pending.clear();
    }

    /// Queue data for the peer. Split to fit N401 (V.42 9.2.3).
    pub fn send_data(&mut self, data: &[u8]) {
        for chunk in data.chunks(self.params.n401) {
            self.pending.push_back(chunk.to_vec());
        }
    }

    /// Advance timers by `dt_ms` (V.42 8.3.2.2, 8.7.3, 8.4.8).
    pub fn tick(&mut self, dt_ms: u32) {
        self.clock_ms += u64::from(dt_ms);
        if let Some(waited) = self.awaiting_ms {
            self.awaiting_ms = Some(waited.saturating_add(dt_ms));
        }
        if self.state == State::Connected {
            self.unheard_ms = self.unheard_ms.saturating_add(dt_ms);
            self.since_probe_ms = self.since_probe_ms.saturating_add(dt_ms);
            // With T401 running, its own expiry polls; this is for the link
            // with nothing of ours outstanding.
            if self.timer.is_none()
                && self.unheard_ms >= PROBE_AFTER_MS
                && self.since_probe_ms >= PROBE_AFTER_MS
            {
                self.probe();
            }
        }
        let Some(remaining) = self.timer else { return };
        if remaining > dt_ms {
            self.timer = Some(remaining - dt_ms);
            return;
        }
        self.timer = None;
        self.timed_out = true;
        self.backoff = (self.backoff + 1).min(16);
        // An expiry is the one thing that says T401 may be too short for this
        // line without saying by how much, so the next wait is half as long
        // again (see `backed_off_t401`). Appendix IV's sum has two terms this
        // end cannot see -- the far end's transmit queue and its processing --
        // and a line it cannot see at all, so the value that works is found
        // rather than calculated.
        self.retries += 1;
        if self.retries >= self.params.n400 {
            self.fail();
            return;
        }
        match self.state {
            State::AwaitingEstablishment => {
                self.send(Frame::Sabme { poll: true }, Kind::Command);
                self.start_timer();
            }
            State::AwaitingRelease => {
                self.send(Frame::Disc { poll: true }, Kind::Command);
                self.start_timer();
            }
            State::Connected => {
                // V.42 8.5.3: enter the timer-recovery condition and poll,
                // rather than blindly resending. The acknowledgement may simply
                // have been lost, and the reply says what is really missing.
                //
                // This is the only way a lost *final* frame is recovered: with
                // nothing following it, the peer never sees anything out of
                // sequence and so never sends a reject.
                self.timer_recovery = true;
                self.send(Frame::Rr { nr: self.vr, pf: true }, Kind::Command);
                self.ack_pending = false;
                self.start_timer();
            }
            State::Disconnected => {}
        }
    }

    /// Handle a frame from the peer.
    pub fn receive(&mut self, frame: Frame, kind: Kind) {
        if let Some(t) = self.trace.as_mut() {
            let what = match &frame {
                Frame::I { ns, nr, poll, info } => format!("I ns {ns} nr {nr} p {poll} len {}", info.len()),
                Frame::Rr { nr, pf } => format!("RR nr {nr} pf {pf} {kind:?}"),
                Frame::Rej { nr, pf } => format!("REJ nr {nr} pf {pf}"),
                other => format!("{other:?}").chars().take(40).collect(),
            };
            t.push(format!("{} in {what} va {} vs {}", self.clock_ms, self.va, self.vs));
        }
        // V.42 8.3.2.1: "Upon receipt of an I frame or a supervisory frame,
        // the originator of the SABME command may assume that the responding
        // error-correcting entity has received and accepted the SABME command
        // and sent a UA response, but that the UA response was lost in
        // transmission. It may proceed as though a UA response has been
        // received, and perform the actions noted above for reception of the
        // UA response before processing the received I frame or supervisory
        // frame." Discarding it instead left a real far end's poll unanswered
        // for the 0.58 s until its UA came in behind it.
        if self.state == State::AwaitingEstablishment
            && matches!(
                frame,
                Frame::I { .. }
                    | Frame::Rr { .. }
                    | Frame::Rnr { .. }
                    | Frame::Rej { .. }
                    | Frame::Srej { .. }
            )
        {
            self.establish();
        }
        match &frame {
            // Set-mode and release commands are handled in every state
            // (V.42 8.8, 8.9).
            Frame::Sabme { poll } => self.on_sabme(*poll),
            Frame::Disc { poll } => self.on_disc(*poll),
            Frame::Ua { final_bit } => self.on_ua(*final_bit),
            Frame::Dm { final_bit } => self.on_dm(*final_bit),
            _ if self.state != State::Connected => {
                // V.42 8.8: everything else is discarded while disconnected.
            }
            Frame::I { ns, nr, poll, info } => self.on_i(*ns, *nr, *poll, info.clone()),
            Frame::Rr { nr, pf } => self.on_rr(*nr, *pf, kind),
            Frame::Rnr { nr, pf } => self.on_rnr(*nr, *pf, kind),
            Frame::Rej { nr, pf } => self.on_rej(*nr, *pf, kind),
            Frame::Srej { nr } => self.on_srej(*nr),
            // SREJ, UI, XID, TEST and FRMR are not implemented yet; ignoring
            // them is safe because the peer's timers will recover.
            _ => {}
        }
    }

    // -- unnumbered handling ------------------------------------------------

    fn on_sabme(&mut self, poll: bool) {
        let was_connected = self.state == State::Connected;
        if was_connected && !self.exchanged {
            // Nothing has been acknowledged or received since the link came
            // up, so the peer is still opening it: its UA was late (a long
            // VoIP round trip outlasts its T401) and it asked again. Both ends
            // are still at the numbering and the compression state they
            // started from, so what this end sent is still exactly what the
            // peer needs, under the same numbers -- and the peer, awaiting its
            // UA, threw the first copies away. Resetting here (8.2.4.3)
            // discards the only copy: the first data of the call, a login
            // screen, never arrived. Answer, and send it all again instead.
            self.out.retain(|(f, _)| !matches!(f, Frame::I { .. }));
            self.send(Frame::Ua { final_bit: poll }, Kind::Response);
            let vr = self.vr;
            for (ns, info) in self.unacked.clone() {
                self.out.push_back((Frame::I { ns, nr: vr, poll: false, info }, Kind::Command));
            }
            self.peer_busy = false;
            self.reject_sent = false;
            self.timer_recovery = false;
            self.retries = 0;
            if self.unacked.is_empty() {
                self.stop_timer();
            } else {
                self.start_timer();
            }
            return;
        }
        let before = was_connected.then(|| PreReset {
            vs: self.vs,
            va: self.va,
            vr: self.vr,
            unacked: self.unacked.clone(),
            held: self.held.clone(),
        });
        // V.42 8.2.4.3: acceptance resets V(S), V(A) and V(R), and previously
        // transmitted unacknowledged I frames are discarded.
        self.reset_variables();
        self.pre_reset = before;
        self.send(Frame::Ua { final_bit: poll }, Kind::Response);
        self.state = State::Connected;
        self.stop_timer();
        if was_connected {
            self.events.push_back(Event::Reset);
        } else {
            self.events.push_back(Event::Connected);
        }
    }

    fn on_disc(&mut self, poll: bool) {
        if self.state == State::Disconnected {
            // V.42 8.8: answer with DM rather than UA.
            self.send(Frame::Dm { final_bit: poll }, Kind::Response);
            return;
        }
        self.send(Frame::Ua { final_bit: poll }, Kind::Response);
        self.enter_disconnected(Cause::Peer);
    }

    fn on_ua(&mut self, final_bit: bool) {
        match self.state {
            State::AwaitingEstablishment if final_bit => self.establish(),
            State::AwaitingRelease if final_bit => self.enter_disconnected(Cause::Local),
            _ => {}
        }
    }

    /// What the originator of a SABME does on its UA (V.42 8.3.2.1): stop
    /// T401, set V(S), V(R) and V(A) to 0, and enter the connected state,
    /// telling the control function with an L-ESTABLISH confirm. T403, which
    /// the clause also starts, is not implemented.
    ///
    /// That list and no more. The responder's list, a few lines earlier in the
    /// same clause, has two entries this one has not -- "clear all existing
    /// exception conditions" and "clear any existing peer-receiver busy
    /// condition" -- and the originator does not need them, because it cleared
    /// them where the clause opens: "a request to establish the error-corrected
    /// connection is initiated by the transmission of the SABME command. All
    /// existing exception conditions shall be cleared, the retransmission
    /// counter shall be reset", which is [`Self::connect`].
    ///
    /// A blanket reset was the same thing in practice while a UA was the only
    /// way in. It is the wrong shape to leave behind now that an I or
    /// supervisory frame standing in for a lost UA comes through the same door
    /// and is processed immediately after: what that frame meets should be
    /// what the clause gives a newly connected originator, and not whatever a
    /// wider reset happens to coincide with.
    fn establish(&mut self) {
        self.exchanged = false;
        self.heard();
        self.since_probe_ms = 0;
        self.stop_timer();
        self.vs = 0;
        self.va = 0;
        self.vr = 0;
        self.state = State::Connected;
        self.events.push_back(Event::Connected);
    }

    fn on_dm(&mut self, final_bit: bool) {
        match self.state {
            // V.42 8.9.3: a DM with F clear colliding with a set-mode command
            // is ignored.
            State::AwaitingEstablishment if final_bit => {
                self.enter_disconnected(Cause::Refused)
            }
            State::AwaitingRelease if final_bit => self.enter_disconnected(Cause::Local),
            State::Connected => self.enter_disconnected(Cause::Refused),
            _ => {}
        }
    }

    // -- numbered handling --------------------------------------------------

    fn on_i(&mut self, ns: u8, nr: u8, poll: bool, info: Vec<u8>) {
        self.acknowledge(nr);
        if let Some(p) = &self.pre_reset {
            if ns != self.vr && ns == p.vr {
                // Numbered from where the peer was before the SABME: it never
                // reset.
                let p = self.pre_reset.take().unwrap();
                self.undo_reset(p);
            } else if ns == self.vr && ns != p.vr {
                // Numbered from 0 again: the peer did reset.
                self.pre_reset = None;
            }
        }
        // A frame numbered behind V(R) is one already taken, sent again
        // because its acknowledgement had not arrived yet -- on a path with
        // a second of delay a modem whose T401 is shorter than that repeats
        // every frame. Nothing is missing, so nothing is asked for: an SREJ
        // here would name V(R), a frame the peer has not sent yet, and a USR
        // Sportster answers that one with the stale contents of its
        // retransmission store. The frame is dropped and acknowledged again.
        let ahead = ns.wrapping_sub(self.vr) % MODULUS;
        // Behind or ahead is told by which half of the numbering it falls in,
        // not by k: the two directions may have agreed different windows.
        let duplicate = ahead >= MODULUS / 2;
        if ns == self.vr {
            self.deliver(info);
        } else if duplicate {
            self.ack_pending = true;
        } else if self.srej {
            // Out of sequence, and the frames after the missing one are worth
            // keeping: only the one that was lost has to be asked for
            // (8.2.4.8.1). Bounded by the window, so that a sequence number
            // from nowhere cannot make this grow.
            let top = (self.vr + u8::min(self.receive_k, MODULUS - 1)) % MODULUS;
            if in_window(self.vr, ns, top) {
                self.held.insert(ns, info);
            }
            if !self.reject_sent {
                self.reject_sent = true;
                // 8.2.4.8.1: the P/F bit of an SREJ is always 0, and its N(R)
                // asks for one frame rather than acknowledging any.
                self.send(Frame::Srej { nr: self.vr }, Kind::Command);
            }
        } else if !self.reject_sent {
            // Out of sequence: ask for everything from V(R) again
            // (V.42 8.4.4). Only one outstanding reject at a time.
            self.reject_sent = true;
            self.send(Frame::Rej { nr: self.vr, pf: false }, Kind::Command);
        }
        if poll {
            // V.42 8.4.2.1: a command with P set demands an immediate response.
            self.send(Frame::Rr { nr: self.vr, pf: true }, Kind::Response);
            self.ack_pending = false;
        }
    }

    /// Take an in-sequence frame, and everything its arrival unblocks.
    ///
    /// The SREJ exception clears here too: 8.2.4.8.1 clears it "upon receipt
    /// of the I frame with an N(S) equal to the N(R) of the SREJ frame", and
    /// that N(R) was V(R), which is the frame being taken.
    fn deliver(&mut self, info: Vec<u8>) {
        self.exchanged = true;
        self.vr = (self.vr + 1) % MODULUS;
        self.reject_sent = false;
        self.ack_pending = true;
        if !info.is_empty() {
            self.events.push_back(Event::Data(info));
        }
        while let Some(next) = self.held.remove(&self.vr) {
            self.vr = (self.vr + 1) % MODULUS;
            if !next.is_empty() {
                self.events.push_back(Event::Data(next));
            }
        }
    }

    /// V.42 8.4.5.1: send again the one I frame the peer asked for.
    fn on_srej(&mut self, nr: u8) {
        self.heard();
        self.peer_busy = false;
        // "The N(R) of the SREJ frame does not indicate acknowledgement of any
        // I frames" (8.2.4.8.1), so V(A) does not move -- which is the whole
        // difference between this and a reject.
        let Some((ns, info)) = self.unacked.iter().find(|(ns, _)| *ns == nr).cloned() else {
            return;
        };
        self.out.push_back((
            Frame::I { ns, nr: self.vr, poll: false, info },
            Kind::Command,
        ));
        self.ack_pending = false;
        self.start_timer();
    }

    fn on_rr(&mut self, nr: u8, pf: bool, kind: Kind) {
        self.heard_final(pf, kind);
        self.peer_busy = false;
        self.acknowledge(nr);
        self.resolve_timer_recovery(pf, kind);
        self.answer_poll(pf, kind);
    }

    fn on_rnr(&mut self, nr: u8, pf: bool, kind: Kind) {
        self.heard_final(pf, kind);
        self.acknowledge(nr);
        self.peer_busy = true;
        self.resolve_timer_recovery(pf, kind);
        self.answer_poll(pf, kind);
        // Busy must remain timed even when all I frames were acknowledged:
        // the peer's later RR can be lost, leaving pending data stuck forever.
        if self.timer.is_none() {
            self.start_timer();
        }
    }

    /// A response with F set answers a poll of ours (8.4.2.1), so the poll
    /// got there.
    fn heard_final(&mut self, pf: bool, kind: Kind) {
        if pf && kind == Kind::Response {
            self.heard();
        }
    }

    /// Act on the reply to a timer-recovery poll (V.42 8.5.3).
    ///
    /// The peer's N(R) has already been applied, so anything still
    /// unacknowledged never arrived and is sent again.
    fn resolve_timer_recovery(&mut self, pf: bool, kind: Kind) {
        if !self.timer_recovery || !pf || kind != Kind::Response {
            return;
        }
        self.timer_recovery = false;
        self.retries = 0;
        if self.va != self.vs && !self.peer_busy {
            self.retransmit_from(self.va);
        }
    }

    fn on_rej(&mut self, nr: u8, pf: bool, kind: Kind) {
        // A reject is the peer reading our frames and finding one missing.
        self.heard();
        self.peer_busy = false;
        self.acknowledge(nr);
        // Go back to N(R): everything still unacknowledged is sent again.
        self.retransmit_from(nr);
        self.answer_poll(pf, kind);
    }

    /// A command with P set requires a response with F set (V.42 8.4.2.1).
    fn answer_poll(&mut self, pf: bool, kind: Kind) {
        if pf && kind == Kind::Command {
            self.send(Frame::Rr { nr: self.vr, pf: true }, Kind::Response);
            self.ack_pending = false;
        }
    }

    /// Retire acknowledged frames. N(R) is valid when V(A) <= N(R) <= V(S)
    /// modulo 128 (V.42 8.2.3.2.3).
    fn acknowledge(&mut self, nr: u8) {
        if !in_window(self.va, nr, self.vs) {
            // Outside the window. After a SABME received while connected, an
            // N(R) that fits the numbering from before it is the peer saying
            // it never reset.
            match self.pre_reset.take() {
                Some(p) if in_window(p.va, nr, p.vs) => self.undo_reset(p),
                _ => return,
            }
            if !in_window(self.va, nr, self.vs) {
                return;
            }
        } else if nr != self.va {
            // It acknowledged a frame sent since the reset: the reset stands.
            self.pre_reset = None;
        }
        let previous = self.va;
        if nr != self.va {
            self.exchanged = true;
            self.heard();
        }
        while self.va != nr {
            self.unacked.pop_front();
            self.va = (self.va + 1) % MODULUS;
        }
        if self.va == self.vs {
            // Everything is acknowledged, so nothing is outstanding to time.
            self.stop_timer();
            self.retries = 0;
        } else if self.va != previous {
            // Some of them were acknowledged and others were not, so the
            // oldest outstanding frame is a different and later one: the wait
            // being measured is its wait, not the retired frame's. The peer
            // answering ends the backoff too: in a bulk transfer the window is
            // never empty, and a backoff left standing until it was would
            // make every later loss cost the ceiling (2026-11-17).
            self.retries = 0;
            if !self.timed_out {
                self.backoff = 0;
            }
            self.awaiting_ms = None;
            self.start_timer();
        } else if self.timer.is_none() {
            // A duplicate N(R) is not progress. Keep the oldest frame's
            // existing deadline so repeated RR cannot suppress recovery.
            self.start_timer();
        }
    }

    fn retransmit_from(&mut self, nr: u8) {
        if !in_window(self.va, nr, self.vs) {
            return;
        }
        // Re-send from the buffer without disturbing V(S): the frames keep the
        // sequence numbers they were first given.
        let frames: Vec<(u8, Vec<u8>)> = self
            .unacked
            .iter()
            .filter(|(ns, _)| in_window(nr, *ns, self.vs))
            .cloned()
            .collect();
        for (ns, info) in frames {
            self.out.push_back((
                Frame::I { ns, nr: self.vr, poll: false, info },
                Kind::Command,
            ));
        }
        self.ack_pending = false;
        self.start_timer();
    }

    /// Hand off one frame at a time. Frames still waiting for the serializer
    /// must not consume the transmit window or start acknowledgement timers.
    fn pump(&mut self) {
        if self.state != State::Connected || !self.out.is_empty() {
            return;
        }
        if !self.peer_busy
            && !self.pending.is_empty()
            && outstanding(self.va, self.vs) < self.params.k
        {
            let info = self.pending.pop_front().expect("checked non-empty");
            let ns = self.vs;
            self.unacked.push_back((ns, info.clone()));
            self.vs = (self.vs + 1) % MODULUS;
            self.out.push_back((
                // N(R) rides along, which acknowledges received frames for free.
                Frame::I { ns, nr: self.vr, poll: false, info },
                Kind::Command,
            ));
            self.ack_pending = false;
            self.start_timer();
        }
        if self.ack_pending {
            self.ack_pending = false;
            self.send(Frame::Rr { nr: self.vr, pf: false }, Kind::Response);
        }
    }

    // -- housekeeping -------------------------------------------------------

    fn send(&mut self, frame: Frame, kind: Kind) {
        self.out.push_back((frame, kind));
    }

    /// Put back the sequence state a duplicate SABME reset (see `pre_reset`).
    ///
    /// Frames sent since the reset went out numbered from 0 and the peer
    /// discarded them as duplicates, so they are renumbered after the old
    /// state and sent again.
    fn undo_reset(&mut self, p: PreReset) {
        let since: Vec<Vec<u8>> = self.unacked.drain(..).map(|(_, info)| info).collect();
        // Anything still queued under the post-reset numbering is wrong too.
        self.out.retain(|(f, _)| !matches!(f, Frame::I { .. }));
        self.vs = p.vs;
        self.va = p.va;
        self.vr = p.vr;
        self.unacked = p.unacked;
        self.held = p.held;
        for info in since {
            let ns = self.vs;
            self.unacked.push_back((ns, info.clone()));
            self.out.push_back((Frame::I { ns, nr: self.vr, poll: false, info }, Kind::Command));
            self.vs = (self.vs + 1) % MODULUS;
        }
        if self.va != self.vs {
            self.start_timer();
        }
        self.events.push_back(Event::ResetUndone);
    }

    fn reset_variables(&mut self) {
        self.pre_reset = None;
        self.exchanged = false;
        self.out.clear();
        self.vs = 0;
        self.va = 0;
        self.vr = 0;
        self.held.clear();
        self.unacked.clear();
        self.peer_busy = false;
        self.reject_sent = false;
        self.ack_pending = false;
        self.timer_recovery = false;
        self.unheard_ms = 0;
        self.since_probe_ms = 0;
    }

    fn enter_disconnected(&mut self, cause: Cause) {
        self.state = State::Disconnected;
        self.stop_timer();
        self.pending.clear();
        self.unacked.clear();
        self.events.push_back(Event::Released(cause));
    }

    fn fail(&mut self) {
        self.enter_disconnected(Cause::NoResponse);
    }

    fn start_timer(&mut self) {
        // A retransmission is the same command still outstanding, so the wait
        // carries on being measured from when it was first asked for.
        if self.awaiting_ms.is_none() {
            self.timed_out = false;
        }
        self.timer = Some(self.backed_off_t401());
        self.awaiting_ms = self.awaiting_ms.or(Some(0));
    }

    fn stop_timer(&mut self) {
        self.timer = None;
        if let Some(waited) = self.awaiting_ms.take()
            && !self.timed_out
        {
            self.backoff = 0;
            // A clean answer, so this is the round trip itself rather than the
            // round trip plus whatever a lost frame cost. Appendix IV asks for
            // at least the sum of the six terms; half again is the margin for
            // the ones that vary.
            let want = waited.saturating_add(waited / 2).min(MAX_T401_MS);
            self.params.t401_ms = self.params.t401_ms.max(want);
        }
    }

    /// The acknowledgement timer as it now stands, in milliseconds.
    ///
    /// The line's part only ever rises: V.42 9.2.1 asks for a timer long
    /// enough to wait for an acknowledgement, and a value that shrank on one
    /// fast frame would spend the next slow one retransmitting. The backoff on
    /// top of it lasts until the peer next answers.
    pub fn t401_ms(&self) -> u32 {
        self.backed_off_t401()
    }

    /// T401 for the next wait: the line's value, half as long again for each
    /// expiry since the peer last answered within the timer, never past
    /// [`MAX_T401_MS`]. The
    /// backoff is kept apart from the line's value so that it ends when the
    /// peer answers; folded into it, a burst of losses early in a call left
    /// every wait after it at the ceiling (2026-11-17: 6 s, against a 0.75 s
    /// round trip).
    fn backed_off_t401(&self) -> u32 {
        (0..self.backoff).fold(self.params.t401_ms, |t, _| (t * 3 / 2).min(MAX_T401_MS))
    }

    /// Address this entity uses when encoding.
    pub fn role(&self) -> Role {
        self.role
    }

    pub fn dlci(&self) -> u8 {
        self.dlci
    }
}

/// Number of outstanding I frames, modulo 128.
fn outstanding(va: u8, vs: u8) -> u8 {
    vs.wrapping_sub(va) % MODULUS
}

/// True when `n` lies in `[low, high]` going forward modulo 128.
fn in_window(low: u8, n: u8, high: u8) -> bool {
    let span = high.wrapping_sub(low) % MODULUS;
    let offset = n.wrapping_sub(low) % MODULUS;
    offset <= span
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::DLCI_DATA;

    #[test]
    fn stalled_transmit_tracks_progress_and_excludes_flow_control() {
        let (mut a, mut b)=pair(); a.connect(); settle(&mut a,&mut b);
        assert_eq!(a.stalled_transmit_ms(),None);
        a.send_data(b"pending");assert!(matches!(a.poll_transmit(),Some((Frame::I{..},_))));
        a.tick(500);assert_eq!(a.stalled_transmit_ms(),Some(500));
        a.receive(Frame::Rr{nr:0,pf:false},Kind::Response);
        a.tick(500);assert_eq!(a.stalled_transmit_ms(),Some(1000));
        a.receive(Frame::Rnr{nr:0,pf:false},Kind::Response);
        assert_eq!(a.stalled_transmit_ms(),None,"peer flow control must not retrain");
        a.receive(Frame::Rr{nr:0,pf:false},Kind::Response);
        assert_eq!(a.stalled_transmit_ms(),Some(1000));
        a.receive(Frame::Rr{nr:1,pf:false},Kind::Response);
        assert_eq!(a.stalled_transmit_ms(),None,"acknowledgement clears stall");
    }

    /// Polls from us that the peer is bound to answer: an RR command, P set.
    fn probes(l: &mut Lapm) -> usize {
        let mut n = 0;
        while let Some((frame, kind)) = l.poll_transmit() {
            if matches!(frame, Frame::Rr { pf: true, .. }) && kind == Kind::Command {
                n += 1;
            }
        }
        n
    }

    #[test]
    fn an_idle_link_is_probed_and_a_peer_that_answers_is_heard() {
        let (mut a, mut b) = connected(false);
        assert_eq!(a.unheard_ms(), Some(0));
        let mut sent = 0;
        for _ in 0..40 {
            a.tick(100);
            // Everything a sends reaches b, and b's answers reach a.
            while let Some((frame, kind)) = a.poll_transmit() {
                if matches!(frame, Frame::Rr { pf: true, .. }) && kind == Kind::Command {
                    sent += 1;
                }
                b.receive(frame, kind);
            }
            deliver(&mut b, &mut a);
        }
        assert!(sent >= 1, "an idle link was never probed");
        assert!(a.unheard_ms().unwrap() < 3000, "the answer to the probe was not taken as being heard: {:?}", a.unheard_ms());
    }

    #[test]
    fn a_peer_that_answers_nothing_goes_unheard() {
        let (mut a, _b) = connected(false);
        let mut sent = 0;
        for _ in 0..120 {
            a.tick(100);
            sent += probes(&mut a);
        }
        assert_eq!(a.unheard_ms(), Some(12_000));
        assert!((3..=4).contains(&sent), "probes every 3 s, got {sent} in 12 s");
        assert!(a.is_connected(), "probing must not tear the link down");
    }

    #[test]
    fn a_probe_on_demand_polls_at_once_and_an_answer_is_heard() {
        let (mut a, mut b) = connected(false);
        a.tick(1000);
        assert_eq!(probes(&mut a), 0, "nothing due yet");
        a.probe();
        let (frame, kind) = a.poll_transmit().expect("probe");
        assert!(matches!(frame, Frame::Rr { pf: true, .. }) && kind == Kind::Command, "{frame:?} {kind:?}");
        b.receive(frame, kind);
        deliver(&mut b, &mut a);
        assert_eq!(a.unheard_ms(), Some(0));
    }

    #[test]
    fn a_peers_own_polls_do_not_show_it_hears_us() {
        let (mut a, _b) = connected(false);
        for _ in 0..50 {
            a.tick(100);
            // The peer polls; a answers. Nothing says the answer arrived.
            a.receive(Frame::Rr { nr: 0, pf: true }, Kind::Command);
            while a.poll_transmit().is_some() {}
        }
        assert_eq!(a.unheard_ms(), Some(5000));
    }

    #[test]
    fn data_the_peer_acknowledges_shows_it_hears_us() {
        let (mut a, mut b) = connected(false);
        for _ in 0..50 {
            a.tick(100);
            b.tick(100);
            a.send_data(b"x");
            settle(&mut a, &mut b);
        }
        assert!(a.unheard_ms().unwrap() <= 100, "{:?}", a.unheard_ms());
    }

    fn pair() -> (Lapm, Lapm) {
        (
            Lapm::new(Role::Originator, DLCI_DATA, Params::default()),
            Lapm::new(Role::Answerer, DLCI_DATA, Params::default()),
        )
    }

    #[test]
    fn queued_bulk_data_does_not_delay_the_next_acknowledgement() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        a.send_data(&vec![0x5a; DEFAULT_N401 * DEFAULT_K as usize]);
        let (first, _) = a.poll_transmit().unwrap();
        assert!(matches!(first, Frame::I { ns: 0, nr: 0, .. }));
        assert_eq!(a.vs, 1, "unsent frames consumed the whole transmit window");
        a.receive(Frame::I { ns: 0, nr: 0, poll: false, info: vec![0x42] }, Kind::Command);
        let (next, _) = a.poll_transmit().unwrap();
        assert!(matches!(next, Frame::I { nr: 1, .. } | Frame::Rr { nr: 1, .. }),
            "received data was not acknowledged in the next frame: {next:?}");
    }

    /// Move every queued frame from `from` to `to`, returning how many crossed.
    fn deliver(from: &mut Lapm, to: &mut Lapm) -> usize {
        let mut n = 0;
        while let Some((frame, kind)) = from.poll_transmit() {
            to.receive(frame, kind);
            n += 1;
        }
        n
    }

    /// Exchange until both ends stop talking.
    fn settle(a: &mut Lapm, b: &mut Lapm) {
        for _ in 0..64 {
            let moved = deliver(a, b) + deliver(b, a);
            if moved == 0 {
                return;
            }
        }
        panic!("the two ends never stopped exchanging frames");
    }

    fn events(l: &mut Lapm) -> Vec<Event> {
        let mut v = Vec::new();
        while let Some(e) = l.poll_event() {
            v.push(e);
        }
        v
    }

    fn data_from(l: &mut Lapm) -> Vec<u8> {
        events(l)
            .into_iter()
            .filter_map(|e| match e {
                Event::Data(d) => Some(d),
                _ => None,
            })
            .flatten()
            .collect()
    }

    /// Move queued frames across, dropping the I frame numbered `lose`.
    ///
    /// One frame lost out of the middle of a full window, which is the case
    /// the two procedures answer differently.
    fn deliver_losing(from: &mut Lapm, to: &mut Lapm, lose: Option<u8>) -> Vec<u8> {
        let mut sent = Vec::new();
        while let Some((frame, kind)) = from.poll_transmit() {
            if let Frame::I { ns, .. } = &frame {
                sent.push(*ns);
                if Some(*ns) == lose {
                    continue;
                }
            }
            to.receive(frame, kind);
        }
        sent
    }

    /// Establish a connection, optionally with selective reject agreed.
    fn connected(srej: bool) -> (Lapm, Lapm) {
        let (mut a, mut b) = pair();
        a.set_selective_reject(srej);
        b.set_selective_reject(srej);
        a.connect();
        settle(&mut a, &mut b);
        assert!(a.is_connected() && b.is_connected());
        (a, b)
    }

    /// Send five frames with the third lost, and report what was sent again.
    fn recover(srej: bool) -> (Vec<u8>, Vec<u8>) {
        let (mut a, mut b) = connected(srej);
        for i in 0..5u8 {
            a.send_data(&[b'a' + i]);
        }
        // The third frame never arrives.
        deliver_losing(&mut a, &mut b, Some(2));
        // Whatever b makes of that goes back, and whatever a sends in reply is
        // the measurement: with go-back-N it is the lost frame and everything
        // after, with selective reject it is the lost frame.
        deliver(&mut b, &mut a);
        let again = deliver_losing(&mut a, &mut b, None);
        settle(&mut a, &mut b);
        (again, data_from(&mut b))
    }

    #[test]
    fn a_reject_asks_for_the_lost_frame_and_everything_after_it() {
        // V.42 8.4.4, which is the procedure without the optional one.
        let (again, data) = recover(false);
        assert_eq!(again, vec![2, 3, 4], "go-back-N resends from the gap");
        assert_eq!(data, b"abcde", "and everything still arrives in order");
    }

    #[test]
    fn a_selective_reject_asks_for_the_lost_frame_and_no_others() {
        // V.42 8.2.4.8.1: "I frames that may have been transmitted following
        // the I frame indicated by the SREJ frame shall not be retransmitted
        // as the result of receiving an SREJ frame." Which is only possible
        // because the receiver kept them, and delivered them in order once the
        // gap closed.
        let (again, data) = recover(true);
        assert_eq!(again, vec![2], "only the frame that was lost");
        assert_eq!(data, b"abcde", "and everything still arrives in order");
    }

    #[test]
    fn frames_after_a_gap_are_held_against_the_far_ends_window_not_ours() {
        // V.42 9.2.4 settles each direction's window on its own: a far end
        // sending 40 outstanding at an end whose own window is 3. The frames
        // after the lost one are in the far end's window, so they are held
        // and only the lost one is sent again.
        let (mut a, mut b) = connected(true);
        a.set_send_limits(1, 40);
        b.set_send_limits(128, 3);
        b.set_receive_window(40);
        let data: Vec<u8> = (0..40u8).collect();
        a.send_data(&data);
        deliver_losing(&mut a, &mut b, Some(2));
        deliver(&mut b, &mut a);
        let again = deliver_losing(&mut a, &mut b, None);
        settle(&mut a, &mut b);
        assert_eq!(again, vec![2], "only the frame that was lost");
        assert_eq!(data_from(&mut b), data);
    }

    #[test]
    fn a_frame_sent_twice_is_dropped_without_asking_for_anything() {
        // A peer whose T401 is shorter than the round trip repeats frames
        // that arrived. Nothing is missing, so neither reject may be sent:
        // it would name a frame the peer has not sent yet.
        for srej in [false, true] {
            let (mut a, mut b) = connected(srej);
            a.send_data(b"a");
            let (frame, kind) = a.poll_transmit().expect("the frame");
            b.receive(frame.clone(), kind);
            while b.poll_transmit().is_some() {}
            b.receive(frame, kind);
            let mut answers = Vec::new();
            while let Some((f, _)) = b.poll_transmit() {
                answers.push(f);
            }
            assert!(
                answers.iter().all(|f| !matches!(f, Frame::Srej { .. } | Frame::Rej { .. })),
                "a duplicate was answered with a reject: {answers:?}"
            );
            assert_eq!(data_from(&mut b), b"a", "and it is delivered once");
            // The next frame is still taken in sequence.
            a.send_data(b"b");
            deliver(&mut a, &mut b);
            assert_eq!(data_from(&mut b), b"b");
        }
    }

    #[test]
    fn a_selective_reject_does_not_acknowledge_anything() {
        // 8.2.4.8.1: "the N(R) of the SREJ frame does not indicate
        // acknowledgement of any I frames". An end that treated it as one
        // would drop the frames before the gap from its retransmission buffer
        // and have nothing to send if they were asked for again.
        let (mut a, mut b) = connected(true);
        for i in 0..4u8 {
            a.send_data(&[b'a' + i]);
        }
        deliver_losing(&mut a, &mut b, Some(1));
        deliver(&mut b, &mut a);
        // Frames 1, 2 and 3 are still outstanding: 1 because it was lost, and
        // 2 and 3 because an SREJ acknowledges nothing.
        assert_eq!(a.va, 1, "V(A) moved on a frame that was never acknowledged");
    }

    #[test]
    fn frames_held_behind_a_gap_are_not_delivered_early() {
        // Out of order on the line is in order at the DTE, or the whole thing
        // is pointless.
        let (mut a, mut b) = connected(true);
        for i in 0..4u8 {
            a.send_data(&[b'a' + i]);
        }
        deliver_losing(&mut a, &mut b, Some(0));
        assert_eq!(data_from(&mut b), b"", "nothing can be delivered yet");
        deliver(&mut b, &mut a);
        deliver_losing(&mut a, &mut b, None);
        assert_eq!(data_from(&mut b), b"abcd", "and then all of it, in order");
    }

    #[test]
    fn window_arithmetic_wraps_at_the_modulus() {
        assert!(in_window(0, 0, 0));
        assert!(in_window(0, 3, 5));
        assert!(!in_window(0, 6, 5));
        // Wrapping across 127 to 0.
        assert!(in_window(126, 1, 3));
        assert!(!in_window(126, 4, 3));
        assert_eq!(outstanding(126, 2), 4);
    }

    #[test]
    fn duplicate_sabme_after_data_does_not_desynchronise() {
        // Seen on a call with ~0.9 s of round-trip delay: the caller's SABME
        // was answered, the answerer sent two I frames, and only then did the
        // caller's retransmitted SABME arrive. Resetting on it put the
        // answerer back at N(S) 0 while the caller, already connected,
        // expected 2 -- and discarded everything after as duplicates.
        let (mut caller, mut answerer) = pair();
        caller.connect();
        deliver(&mut caller, &mut answerer); // SABME
        deliver(&mut answerer, &mut caller); // UA
        assert!(caller.is_connected() && answerer.is_connected());

        answerer.send_data(b"screen ");
        answerer.send_data(b"Login: ");
        deliver(&mut answerer, &mut caller); // I(0), I(1)
        assert_eq!(data_from(&mut caller), b"screen Login: ");

        // The late copy of the SABME.
        answerer.receive(Frame::Sabme { poll: true }, Kind::Command);
        deliver(&mut answerer, &mut caller); // a UA the caller has no use for
        settle(&mut caller, &mut answerer);  // the caller's RR N(R)=2

        answerer.send_data(b"cooper\r\nPassword: ");
        settle(&mut answerer, &mut caller);
        assert_eq!(data_from(&mut caller), b"cooper\r\nPassword: ",
            "data after a duplicate SABME must still arrive");

        // And it keeps working in both directions afterwards.
        caller.send_data(b"test\r");
        answerer.send_data(b"Entering PPP Session.");
        settle(&mut answerer, &mut caller);
        assert_eq!(data_from(&mut answerer), b"test\r");
        assert_eq!(data_from(&mut caller), b"Entering PPP Session.");
    }

    #[test]
    fn data_sent_between_a_duplicate_sabme_and_its_proof_is_resent() {
        // The answerer may already have sent new frames, numbered from 0,
        // before the caller's acknowledgement shows the reset was spurious.
        let (mut caller, mut answerer) = pair();
        caller.connect();
        settle(&mut caller, &mut answerer);
        answerer.send_data(b"one ");
        answerer.send_data(b"two ");
        deliver(&mut answerer, &mut caller);
        answerer.receive(Frame::Sabme { poll: true }, Kind::Command);
        answerer.send_data(b"three");          // goes out as N(S) 0: discarded
        deliver(&mut answerer, &mut caller);
        settle(&mut caller, &mut answerer);
        assert_eq!(data_from(&mut caller), b"one two three",
            "the frame sent under the wrong number is sent again, renumbered");
    }

    /// The 2026-11-02 V.92 calls: over a 0.9 s VoIP round trip the caller's
    /// T401 ran out before the UA reached it, so it sent its SABME again --
    /// after the answerer had already sent the login screen as I frame 0.
    /// The caller, still awaiting its UA, had discarded that frame, and the
    /// reset on the second SABME discarded the only copy: the caller polled
    /// with RR N(R)=0 for the rest of the call and the screen never came.
    #[test]
    fn a_caller_still_opening_the_link_gets_what_was_sent_before_its_second_sabme() {
        let (mut caller, mut answerer) = pair();
        caller.connect();
        deliver(&mut caller, &mut answerer); // SABME
        answerer.send_data(b"Login:");
        while answerer.poll_transmit().is_some() {} // UA and I 0: too late, not taken
        caller.tick(Params::default().t401_ms + 1); // so the caller asks again
        deliver(&mut caller, &mut answerer);
        settle(&mut caller, &mut answerer);
        assert_eq!(data_from(&mut caller), b"Login:");
        assert!(!events(&mut answerer).contains(&Event::Reset),
            "nothing was exchanged, so there was nothing to reset");
    }

    /// Once anything has been acknowledged the SABME is a real reset again
    /// (8.2.4.3), whatever was outstanding.
    #[test]
    fn a_sabme_after_data_has_flowed_still_resets() {
        let (mut caller, mut answerer) = pair();
        caller.connect();
        settle(&mut caller, &mut answerer);
        answerer.send_data(b"one");
        settle(&mut caller, &mut answerer);
        events(&mut answerer);
        answerer.receive(Frame::Sabme { poll: true }, Kind::Command);
        assert!(events(&mut answerer).contains(&Event::Reset));
    }

    #[test]
    fn an_undone_reset_is_reported() {
        let (mut caller, mut answerer) = pair();
        caller.connect();
        settle(&mut caller, &mut answerer);
        // Something acknowledged first: before that a SABME is the caller
        // still opening the link and resets nothing at all.
        answerer.send_data(b"w");
        settle(&mut caller, &mut answerer);
        events(&mut answerer);
        answerer.send_data(b"x");
        deliver(&mut answerer, &mut caller);
        answerer.receive(Frame::Sabme { poll: true }, Kind::Command);
        deliver(&mut answerer, &mut caller);
        settle(&mut caller, &mut answerer);
        let ev = events(&mut answerer);
        assert!(ev.contains(&Event::Reset));
        assert!(ev.contains(&Event::ResetUndone),
            "the stack has to know, to resynchronise compression: {ev:?}");
    }

    #[test]
    fn sabme_and_ua_establish_a_connection() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        assert!(a.is_connected());
        assert!(b.is_connected());
        assert!(events(&mut a).contains(&Event::Connected));
        assert!(events(&mut b).contains(&Event::Connected));
    }

    #[test]
    fn data_crosses_in_order() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        events(&mut b);

        a.send_data(b"the quick brown fox");
        settle(&mut a, &mut b);
        assert_eq!(data_from(&mut b), b"the quick brown fox");
    }

    #[test]
    fn data_flows_in_both_directions() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        events(&mut a);
        events(&mut b);

        a.send_data(b"from a");
        b.send_data(b"from b");
        settle(&mut a, &mut b);
        assert_eq!(data_from(&mut b), b"from a");
        assert_eq!(data_from(&mut a), b"from b");
    }

    #[test]
    fn long_data_is_split_to_n401_and_reassembles() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        events(&mut b);

        let payload: Vec<u8> = (0..1000).map(|i| (i % 251) as u8).collect();
        a.send_data(&payload);
        settle(&mut a, &mut b);
        assert_eq!(data_from(&mut b), payload);
    }

    #[test]
    fn the_window_limits_frames_in_flight() {
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, Params { k: 3, n401: 1, ..Default::default() });
        let mut b = Lapm::new(Role::Answerer, DLCI_DATA, Params { k: 3, n401: 1, ..Default::default() });
        a.connect();
        settle(&mut a, &mut b);

        a.send_data(b"abcdefgh");
        // Collect without delivering, so nothing is acknowledged.
        let mut sent = 0;
        while let Some((frame, _)) = a.poll_transmit() {
            if matches!(frame, Frame::I { .. }) {
                sent += 1;
            }
        }
        assert_eq!(sent, 3, "k is 3, so only three frames may be outstanding");
    }

    #[test]
    fn a_lost_frame_is_recovered_by_reject() {
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, Params { n401: 1, ..Default::default() });
        let mut b = Lapm::new(Role::Answerer, DLCI_DATA, Params { n401: 1, ..Default::default() });
        a.connect();
        settle(&mut a, &mut b);
        events(&mut b);

        a.send_data(b"abcde");
        // Drop the second I frame on the way across.
        let mut index = 0;
        while let Some((frame, kind)) = a.poll_transmit() {
            let is_i = matches!(frame, Frame::I { .. });
            if is_i {
                index += 1;
                if index == 2 {
                    continue; // lost in transit
                }
            }
            b.receive(frame, kind);
        }
        settle(&mut a, &mut b);
        assert_eq!(
            data_from(&mut b),
            b"abcde",
            "go-back-N should have replaced the lost frame"
        );
    }

    #[test]
    fn duplicate_acknowledgements_cannot_postpone_loss_recovery_forever() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        a.send_data(b"lost data");
        while a.poll_transmit().is_some() {} // lose the I frame
        let timeout = a.t401_ms();
        for _ in 0..4 {
            a.tick(timeout / 4 + 1);
            a.receive(Frame::Rr { nr: 0, pf: false }, Kind::Response);
        }
        assert!(a.timer_recovery, "duplicate RR kept restarting the loss timer");
        settle(&mut a, &mut b);
        assert_eq!(data_from(&mut b), b"lost data");
    }

    #[test]
    fn a_lost_ready_notification_is_recovered_by_polling_a_busy_peer() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        a.receive(Frame::Rnr { nr: 0, pf: false }, Kind::Command);
        a.send_data(b"resume data");
        while a.poll_transmit().is_some() {}
        // The peer is ready again, but its unsolicited RR was lost.
        a.tick(a.t401_ms() + 1);
        settle(&mut a, &mut b);
        assert_eq!(data_from(&mut b), b"resume data");
    }

    #[test]
    fn receive_not_ready_stops_the_sender() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);

        a.receive(Frame::Rnr { nr: 0, pf: false }, Kind::Command);
        a.send_data(b"blocked");
        let mut sent_i = 0;
        while let Some((frame, _)) = a.poll_transmit() {
            if matches!(frame, Frame::I { .. }) {
                sent_i += 1;
            }
        }
        assert_eq!(sent_i, 0, "RNR should hold everything back");

        // RR releases it again.
        a.receive(Frame::Rr { nr: 0, pf: false }, Kind::Command);
        settle(&mut a, &mut b);
        assert!(!data_from(&mut b).is_empty());
    }

    #[test]
    fn disconnect_is_confirmed_by_the_peer() {
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        events(&mut a);
        events(&mut b);

        a.disconnect();
        settle(&mut a, &mut b);
        assert_eq!(a.state(), State::Disconnected);
        assert_eq!(b.state(), State::Disconnected);
        assert!(events(&mut a).contains(&Event::Released(Cause::Local)));
        assert!(events(&mut b).contains(&Event::Released(Cause::Peer)));
    }

    #[test]
    fn disc_while_disconnected_is_answered_with_dm() {
        // V.42 8.8.
        let mut a = Lapm::new(Role::Answerer, DLCI_DATA, Params::default());
        a.receive(Frame::Disc { poll: true }, Kind::Command);
        let (frame, kind) = a.poll_transmit().expect("should have answered");
        assert_eq!(frame, Frame::Dm { final_bit: true });
        assert_eq!(kind, Kind::Response);
    }

    #[test]
    fn establishment_retries_then_gives_up_after_n400() {
        let params = Params { n400: 3, t401_ms: 1000, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        a.connect();
        // The first SABME plus two retries, then failure on the third expiry.
        let mut sabmes = 0;
        while let Some((frame, _)) = a.poll_transmit() {
            if matches!(frame, Frame::Sabme { .. }) {
                sabmes += 1;
            }
        }
        for _ in 0..3 {
            // However long the timer has grown to by now: the point of the
            // test is how many attempts there are, not how long they take.
            a.tick(a.t401_ms());
            while let Some((frame, _)) = a.poll_transmit() {
                if matches!(frame, Frame::Sabme { .. }) {
                    sabmes += 1;
                }
            }
        }
        assert_eq!(sabmes, 3, "one initial SABME and N400-1 retries");
        assert_eq!(a.state(), State::Disconnected);
        assert!(events(&mut a).contains(&Event::Released(Cause::NoResponse)));
    }

    /// A far end further away than the timer allows for is still reached.
    ///
    /// The line this was found on runs a 1.28 s round trip, and T401 at
    /// 2400 bit/s came out at 966 ms: the second SABME was always on its way
    /// out before the first one's UA arrived, and a far end that honours the
    /// second resets the sequence variables under a link that was working.
    #[test]
    fn a_timer_too_short_for_the_line_grows_until_it_is_not() {
        let params = Params { t401_ms: 966, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        a.connect();
        let _ = a.poll_transmit();

        // Nothing has come back after one T401, so it was too short.
        a.tick(966);
        assert!(a.t401_ms() > 1280, "still shorter than the round trip");

        // The UA arrives 1.28 s after the SABME first went out. With the
        // longer timer this end is still waiting for it rather than having
        // given up and asked again.
        a.tick(1280 - 966);
        assert_eq!(a.state(), State::AwaitingEstablishment);
        a.receive(Frame::Ua { final_bit: true }, Kind::Response);
        assert_eq!(a.state(), State::Connected);
    }

    /// An outage long enough for T401 to back off to its ceiling.
    fn backed_off_in_an_outage() -> (Lapm, Lapm) {
        let params = Params { t401_ms: 1170, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        let mut b = Lapm::new(Role::Answerer, DLCI_DATA, Params::default());
        a.connect();
        settle(&mut a, &mut b);
        a.send_data(&vec![0x5a; DEFAULT_N401 * 2 * DEFAULT_K as usize]);
        for _ in 0..5 {
            while a.poll_transmit().is_some() {} // the line is gone
            a.tick(a.t401_ms());
        }
        while a.poll_transmit().is_some() {}
        assert!(a.is_connected());
        assert_eq!(a.t401_ms(), MAX_T401_MS, "the backoff during the outage");
        // The line is back; the poll the next expiry sends finds the peer.
        a.tick(a.t401_ms());
        (a, b)
    }

    /// 2026-11-17, a 1 MB download from the CX93001 over a 0.75 s round
    /// trip: a few bursts of line errors early on backed T401 off to its 6 s
    /// ceiling, where it stayed, and every lost final frame after that cost
    /// 6 s plus the round trip -- stalls of 7.3-8.5 s and twice that, 218 B/s
    /// over five minutes. The backoff is for the frames still unanswered;
    /// once the peer answers, the timer is the line's again.
    #[test]
    fn a_backed_off_timer_comes_back_once_the_peer_answers() {
        let (mut a, mut b) = backed_off_in_an_outage();
        settle(&mut a, &mut b);
        assert_eq!(a.va, a.vs, "everything got across once the line was back");
        assert_eq!(a.t401_ms(), 1170);
    }

    /// The same in a bulk transfer, where the window is never empty and
    /// "everything acknowledged" may not come for the rest of the call.
    #[test]
    fn progress_with_frames_still_outstanding_ends_the_backoff() {
        let (mut a, mut b) = backed_off_in_an_outage();
        deliver(&mut a, &mut b); // the poll
        deliver(&mut b, &mut a); // its answer: everything is sent again
        let mut across = |a: &mut Lapm, b: &mut Lapm, n: usize| {
            for _ in 0..n {
                let (frame, kind) = a.poll_transmit().expect("a frame to send");
                b.receive(frame, kind);
            }
            deliver(b, a);
        };
        // The first acknowledgement answers frames that timed out, so it says
        // nothing about the line (Karn's rule); the next one is clean.
        across(&mut a, &mut b, 3);
        across(&mut a, &mut b, 3);
        assert!(a.va == 6 && a.va != a.vs, "va {} vs {}", a.va, a.vs);
        assert_eq!(a.t401_ms(), 1170);
    }

    /// A measurement that arrives cleanly is used as it stands.
    #[test]
    fn a_slow_answer_on_the_first_attempt_lengthens_the_timer() {
        let params = Params { t401_ms: 4000, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        a.connect();
        let _ = a.poll_transmit();
        a.tick(3000);
        a.receive(Frame::Ua { final_bit: true }, Kind::Response);
        // Appendix IV's sum, plus half again for the terms that vary.
        assert_eq!(a.t401_ms(), 4500);
    }

    #[test]
    fn the_timer_does_not_fire_early() {
        let params = Params { t401_ms: 1000, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        a.connect();
        let _ = a.poll_transmit();
        a.tick(400);
        a.tick(400);
        assert!(a.poll_transmit().is_none(), "fired before T401 elapsed");
        a.tick(400);
        assert!(a.poll_transmit().is_some(), "should have retried by now");
    }

    #[test]
    fn a_refused_connection_is_reported() {
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, Params::default());
        a.connect();
        a.receive(Frame::Dm { final_bit: true }, Kind::Response);
        assert_eq!(a.state(), State::Disconnected);
        assert!(events(&mut a).contains(&Event::Released(Cause::Refused)));
    }

    #[test]
    fn an_unsolicited_dm_during_establishment_is_ignored() {
        // V.42 8.9.3: a DM with F clear colliding with SABME is ignored.
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, Params::default());
        a.connect();
        a.receive(Frame::Dm { final_bit: false }, Kind::Response);
        assert_eq!(a.state(), State::AwaitingEstablishment);
    }

    /// V.42 8.3.2.1: an I or supervisory frame while the SABME is unanswered
    /// stands in for a UA that was lost, and is then acted on as usual.
    ///
    /// From `live-1789647424.wav`: the far end's RR poll arrived 0.58 s
    /// before its UA, and was dropped as if nothing were being established.
    #[test]
    fn a_poll_that_arrives_before_the_ua_establishes_and_is_answered() {
        let params = Params { t401_ms: 1000, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        a.send_data(b"queued before the link");
        a.connect();
        let _ = std::iter::from_fn(|| a.poll_transmit()).count();

        a.receive(Frame::Rr { nr: 0, pf: true }, Kind::Command);
        assert_eq!(a.state(), State::Connected);
        assert_eq!(events(&mut a), vec![Event::Connected], "no L-ESTABLISH confirm");
        let sent: Vec<_> = std::iter::from_fn(|| a.poll_transmit()).collect();
        assert!(
            sent.contains(&(Frame::Rr { nr: 0, pf: true }, Kind::Response)),
            "the poll was not answered: {sent:?}"
        );
        assert!(
            sent.contains(&(Frame::I { ns: 0, nr: 0, poll: false, info: b"queued before the link".to_vec() }, Kind::Command)),
            "V(S) did not start from 0: {sent:?}"
        );

        // T401 was stopped for the SABME: nothing is sent again for it, and
        // the UA that follows changes nothing.
        a.receive(Frame::Ua { final_bit: true }, Kind::Response);
        assert_eq!(a.state(), State::Connected);
        assert!(events(&mut a).is_empty());
        a.receive(Frame::Rr { nr: 1, pf: false }, Kind::Response);
        a.tick(10_000);
        let later: Vec<_> = std::iter::from_fn(|| a.poll_transmit()).collect();
        assert!(
            !later.iter().any(|(f, _)| matches!(f, Frame::Sabme { .. })),
            "the SABME was sent again: {later:?}"
        );
    }

    #[test]
    fn an_i_frame_that_arrives_before_the_ua_is_delivered() {
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, Params::default());
        a.connect();
        let _ = std::iter::from_fn(|| a.poll_transmit()).count();
        a.receive(Frame::I { ns: 0, nr: 0, poll: false, info: b"login: ".to_vec() }, Kind::Command);
        assert!(a.is_connected());
        assert_eq!(data_from(&mut a), b"login: ");
    }

    #[test]
    fn only_i_and_supervisory_frames_stand_in_for_a_lost_ua() {
        for frame in [
            Frame::Ui { pf: false, info: b"x".to_vec() },
            Frame::Xid { pf: false, info: vec![0x82] },
            Frame::Test { pf: false, info: vec![] },
            Frame::Ua { final_bit: false },
            Frame::Dm { final_bit: false },
        ] {
            let mut a = Lapm::new(Role::Originator, DLCI_DATA, Params::default());
            a.connect();
            a.receive(frame.clone(), Kind::Response);
            assert_eq!(a.state(), State::AwaitingEstablishment, "{frame:?} established");
        }
        // And nothing stands in for anything when nothing was asked for.
        let mut idle = Lapm::new(Role::Answerer, DLCI_DATA, Params::default());
        idle.receive(Frame::Rr { nr: 0, pf: true }, Kind::Command);
        assert_eq!(idle.state(), State::Disconnected);
        assert!(idle.poll_transmit().is_none());
    }

    #[test]
    fn re_establishment_resets_the_sequence_numbers() {
        // V.42 8.2.4.3: accepting SABME sets V(S), V(A) and V(R) to zero and
        // discards unacknowledged frames.
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);
        events(&mut a);
        events(&mut b);

        a.send_data(b"some data first");
        settle(&mut a, &mut b);
        events(&mut b);

        b.connect(); // peer re-establishes mid-connection
        settle(&mut a, &mut b);
        assert!(events(&mut a).contains(&Event::Reset));

        // Numbering starts again, and traffic still works.
        a.send_data(b"after reset");
        settle(&mut a, &mut b);
        assert_eq!(data_from(&mut b), b"after reset");
    }

    #[test]
    fn a_poll_is_answered_with_a_final() {
        // V.42 8.4.2.1.
        let (mut a, mut b) = pair();
        a.connect();
        settle(&mut a, &mut b);

        b.receive(Frame::Rr { nr: 0, pf: true }, Kind::Command);
        let replies: Vec<_> = std::iter::from_fn(|| b.poll_transmit()).collect();
        assert!(
            replies
                .iter()
                .any(|(f, k)| matches!(f, Frame::Rr { pf: true, .. }) && *k == Kind::Response),
            "expected an RR response with F set, got {replies:?}"
        );
    }

    #[test]
    fn a_lost_final_frame_is_recovered_by_the_timer() {
        // Nothing follows the last frame, so the peer never sees anything out
        // of sequence and never sends a reject. Only T401 recovery gets it back
        // (V.42 8.5.3). This is the case a reject-only implementation loses.
        let params = Params { t401_ms: 1000, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        let mut b = Lapm::new(Role::Answerer, DLCI_DATA, params);
        a.connect();
        settle(&mut a, &mut b);
        events(&mut b);

        a.send_data(b"the last frame goes missing");
        // Drop everything a sends: the single I frame never arrives.
        while a.poll_transmit().is_some() {}
        assert!(data_from(&mut b).is_empty());

        // T401 expires, a polls, b answers with what it actually has.
        a.tick(1000);
        settle(&mut a, &mut b);
        assert_eq!(
            data_from(&mut b),
            b"the last frame goes missing",
            "timer recovery should have replaced the lost frame"
        );
    }

    #[test]
    fn timer_recovery_does_not_resend_what_was_already_acknowledged() {
        let params = Params { t401_ms: 1000, n401: 4, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        let mut b = Lapm::new(Role::Answerer, DLCI_DATA, params);
        a.connect();
        settle(&mut a, &mut b);
        events(&mut b);

        a.send_data(b"abcdefghijkl");
        settle(&mut a, &mut b);
        let first = data_from(&mut b);
        assert_eq!(first, b"abcdefghijkl");

        // A spurious expiry with nothing outstanding must not duplicate data.
        a.tick(1000);
        settle(&mut a, &mut b);
        assert!(
            data_from(&mut b).is_empty(),
            "acknowledged data was sent a second time"
        );
    }

    #[test]
    fn sequence_numbers_wrap_past_the_modulus() {
        let params = Params { n401: 1, ..Default::default() };
        let mut a = Lapm::new(Role::Originator, DLCI_DATA, params);
        let mut b = Lapm::new(Role::Answerer, DLCI_DATA, params);
        a.connect();
        settle(&mut a, &mut b);
        events(&mut b);

        // Well past 128 single-octet frames, so N(S) wraps more than once.
        let payload: Vec<u8> = (0..300).map(|i| (i % 251) as u8).collect();
        a.send_data(&payload);
        settle(&mut a, &mut b);
        assert_eq!(data_from(&mut b), payload, "wrapping lost or reordered data");
    }
}
