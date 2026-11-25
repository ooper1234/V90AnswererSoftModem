//! V.92 PCM upstream, the digital modem's side (9.5.1, 9.6.1): Phase 3 from
//! Ru to E1u, Phase 4 from TRN2d to B1d, and data mode with the PCM upstream
//! receiver. Downstream is V.90's, sent by `digital`'s `Source`. Only reached
//! when INFO1a asked for PCM upstream (Table 18), which this end invites only
//! with `V92_PCM_UPSTREAM=1`.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// The design's result, when its thread is done: the cut taps and the design.
type DesignSlot = Arc<Mutex<Option<(usize, Result<Design, design::DesignError>, f64, String)>>>;

use super::digital::{Out, Settings, Source, Status, FS};
use super::encoder::Mapping;
use super::sequences::{self, Cp, Descriptor};
use super::ucode::Law;
use crate::v34::phase2::Role;
use crate::v34::training::RetrainWatch;
use crate::v92::phase3::{JaDescriptor, Jp, UpJd};
use crate::v92::phase4::{self, CpKind, Cpd, Suvd, UpCp, UpInfo};
use crate::v92::upstream::design::{self, Design, Limits, Target};
use crate::v92::upstream::params::UpParams;
use crate::v92::upstream::receiver::{Decoding, Receiver};
use crate::v92::upstream::training::{Heard, Listener, SuTurn};
use crate::v92::PcmUpstreamRequest;

/// TRN1u's symbols the Listener trains on.
const TRAIN: usize = 2040;
/// Jp's su_extension when the phase could not be measured: half a symbol,
/// which with S̄u's 24.5T puts the analogue modem's symbols a whole symbol on.
const SU_EXTENSION: u16 = 0x8000;
/// Samples of Su taken in after its return from S̄u before its phase is
/// measured (9.5.1.1.7), and over how many of the last it is.
const SU_SETTLE: usize = 192;
const SU_MEASURE: usize = 96;
/// B1u is 48 data frames.
const B1U_FRAMES: usize = 48;
/// Received samples kept for the data-mode receivers to start from.
const RECENT: usize = 8192;
/// Samples a B1u read position catches up by in one sample, beyond the one
/// that came. Each waits a quarter second to learn its correction and then
/// has that much to read; twenty of them reading it in one sample was a
/// stall of tens of milliseconds the line heard. At this pace the backlog
/// is gone in a few hundredths of a second and never near RECENT.
const CATCH_UP: usize = 48;
/// TRN2d's least length in a rate renegotiation, in seconds; V92_RENEG_TRN2D
/// overrides it. 9.8.1.1.2 bounds TRN2d at 16008T (2.001 s) and sets no
/// minimum. Ended once TRN2u was measured, it ran 0.83 s in renegotiations
/// the CX93001 began -- a round trip shorter than in ours -- and that modem
/// was deaf to the data after 14 of 17 of them (2026-11-16); after 1.67 s,
/// in ours, it heard.
pub(super) const RENEG_TRN2D_DEFAULT: f64 = 1.6;

/// TRN2u symbols let go by before measuring, while TRN2d's echo settles, how
/// many are measured, and how long phase 4 waits for them, in seconds, before
/// going on with the TRN1u design: the analogue modem sends SUVu of its own
/// accord after 12000T (9.6.2.1.1).
const TRN2U_SKIP: usize = 240;
const TRN2U_MEASURE: usize = 4096;
const TRN2U_WAIT: f64 = 1.2;
/// Noise power this many times the first TRN1u's, heard while this end was
/// silent, is taken for this end's echo. On the 2026-11-11 call with the
/// ATA's output 3 dB down the rise was 1.36 and 1.5 missed it.
const ECHO_RISE: f64 = 1.15;

/// Whether to design for a line with this end's echo on it. V92_PCM_ECHO=1
/// or 0 says so outright, for a line known to end in a hybrid or known not
/// to; otherwise noise well over the first TRN1u's, heard with this end
/// silent, is taken for the echo left after cancelling. That comparison
/// is close on a line whose own noise grows with the signal.
fn echo_on_line(quiet: Option<f64>, noise: f64) -> bool {
    match std::env::var("V92_PCM_ECHO").as_deref() {
        Ok("1") => true,
        Ok("0") => false,
        _ => quiet.is_some_and(|q| noise > ECHO_RISE * q),
    }
}
/// Samples falling further than this many design sigmas from their points
/// count as poor; how many renegotiations this end will begin for it in one
/// call, and the spacing each adds, in dB.
const POOR: f64 = 1.5;
const MAX_RENEGS: u32 = 2;
const RENEG_MARGIN_DB: f64 = 1.5;
/// The longest sequence looked for, in bits; past it a start is given up.
/// Ja's DIL descriptor is the longest (Table 20 on 8.3.1/V.90): 188 + beta
/// + 17 N/2 + 51 bits, beta up to 2 x 8 x 17 and N up to 255, about 2700; a
/// real Conexant's is 1770.
const LONGEST: usize = 4096;

/// What the wait for Su is given beyond the Recommendation's 5100 ms.
const SU_GRACE: f64 = 2.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    /// Ru, TRN1u and Ja.
    ReadJa,
    /// Sd to Jd, then Su and its turns; Jp, Jp'.
    SendJd,
    /// The second TRN1u and CPt, during DIL or SCR.
    ReadCpt,
    /// Ri until E1u (or, with no DIL, R-bar-i at once).
    AwaitE1u,
    /// TRN2d, SUVd, CPd, Ed, B1d; SUVu, CPu, E2u.
    Phase4,
    /// B1u and the choice of where it is read.
    B1u,
    Data,
    /// A rate renegotiation (9.8): Rd going out or Ru coming in, until the
    /// Ru-to-R-bar-u transition puts both ends back in Phase 4's exchange.
    Reneg,
    Finished,
}

/// A data-mode receiver started at one candidate sample.
#[derive(Clone)]
struct Candidate {
    rx: Receiver,
    /// The symbol, in the Listener's count, its first data frame starts on.
    start: usize,
    next: usize,
    out: Vec<bool>,
    /// Its receiver has learnt its correction from the first samples.
    warmed: bool,
}

/// Bits read from the Listener with the symbol each came from, and how far
/// sequences have been looked for in them.
#[derive(Default, Clone)]
struct Bits {
    bits: Vec<(usize, bool)>,
    scan: usize,
    /// How many bits there were when the scan last came up empty: nothing
    /// is looked for again until more arrive.
    tried: usize,
}

impl Bits {
    fn extend(&mut self, more: Vec<(usize, bool)>) {
        self.bits.extend(more);
    }

    fn clear(&mut self) {
        self.bits.clear();
        self.scan = 0;
        self.tried = 0;
    }

    /// Bits from symbol `from` on are being read again.
    fn drop_from(&mut self, from: usize) {
        let keep = self.bits.iter().take_while(|(s, _)| *s < from).count();
        self.bits.truncate(keep);
        self.scan = self.scan.min(keep);
        self.tried = 0;
    }

    fn plain(&self, from: usize) -> Vec<bool> {
        self.bits[from.min(self.bits.len())..].iter().map(|b| b.1).collect()
    }

    /// The next sequence `parse` finds, its first bit's index and its length.
    fn next<T>(&mut self, parse: impl Fn(&[bool]) -> Option<T>, len: impl Fn(&T) -> usize) -> Option<(T, usize, usize)> {
        if self.bits.len() == self.tried {
            return None;
        }
        let plain = self.plain(0);
        while self.scan < plain.len() {
            if let Some(found) = parse(&plain[self.scan..]) {
                let at = self.scan;
                let n = len(&found);
                self.scan += n;
                return Some((found, at, n));
            }
            // Every sequence starts on frame sync, 17 ones then a 0: a start
            // without it is no start, whatever follows.
            let ones = sequences::SYNC_ONES;
            let synced = plain.len() - self.scan > ones
                && plain[self.scan..self.scan + ones].iter().all(|&b| b)
                && !plain[self.scan + ones];
            if plain.len() - self.scan > ones && !synced || plain.len() - self.scan > LONGEST {
                self.scan += 1;
            } else {
                self.tried = plain.len();
                return None;
            }
        }
        self.tried = plain.len();
        None
    }

    /// Zeros from bit `at` on, and whether a one has ended them.
    fn zeros_from(&self, at: usize) -> (usize, bool) {
        let run = self.bits[at.min(self.bits.len())..].iter().take_while(|b| !b.1).count();
        (run, at + run < self.bits.len())
    }

    fn symbol(&self, at: usize) -> Option<usize> {
        self.bits.get(at).map(|b| b.0)
    }
}

/// CPt and CPu are V.90's CP but for the first two blocks; downstream they
/// mean the same.
fn to_cp(c: &UpCp) -> Cp {
    Cp {
        data_mode: c.kind == CpKind::Data,
        drn: c.drn,
        silence: false,
        redundancy: c.redundancy,
        acknowledge: c.acknowledge,
        a_law: c.a_law,
        upstream_rates: 0,
        lookahead: c.lookahead,
        trn1d_ratio: c.trn1d_ratio,
        shaping: c.shaping,
        intervals: c.intervals,
        constellations: c.constellations.clone(),
        codec: c.codec.clone(),
    }
}

/// Upstream data frames are 12 symbols (8.7.1), V.90's downstream ones 6.
const UP_FRAME: usize = 12;

fn ceil12(n: usize) -> usize {
    n.div_ceil(UP_FRAME) * UP_FRAME
}

/// Ru in data mode (9.8.1.2): its six-symbol pattern arrives as a signal
/// that repeats every six samples and inverts every three, which data does
/// not; then the transition to R-bar-u, where six samples on it is its own
/// opposite.
#[derive(Clone, Default)]
struct RuWatch {
    recent: VecDeque<f64>,
    held: usize,
    heard: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RuEvent {
    Heard,
    /// The first R-bar-u symbol's response reached the codec about four
    /// samples ago.
    Reversal,
}

impl RuWatch {
    const WINDOW: usize = 48;
    const HELD: usize = 96;

    fn feed(&mut self, y: f64) -> Option<RuEvent> {
        self.recent.push_back(y);
        if self.recent.len() > Self::WINDOW + 6 {
            self.recent.pop_front();
        }
        let n = self.recent.len();
        if n < Self::WINDOW + 6 {
            return None;
        }
        let r = &self.recent;
        if !self.heard {
            let (mut energy, mut three) = (0.0, 0.0);
            for i in 6..n {
                energy += r[i] * r[i];
                three += r[i] * r[i - 3];
            }
            // Loud enough to be a signal at all, and inverting every three.
            if energy > Self::WINDOW as f64 * 2500.0 && three < -0.8 * energy {
                self.held += 1;
            } else {
                self.held = 0;
            }
            if self.held >= Self::HELD {
                self.heard = true;
                return Some(RuEvent::Heard);
            }
            return None;
        }
        let (mut energy, mut six) = (0.0, 0.0);
        for i in n - 6..n {
            energy += r[i] * r[i];
            six += r[i] * r[i - 6];
        }
        (six < -0.5 * energy).then_some(RuEvent::Reversal)
    }
}

#[derive(Clone)]
pub(super) struct Machine {
    /// Whether the Listener is being fed. It is not in data mode; when a
    /// renegotiation starts it again, a whole number of data frames is left
    /// out, so its symbol count still marks the frames (9.8: "shall maintain
    /// data frame synchronization").
    listening: bool,
    /// Samples the Listener has been given, and how many it was not.
    fed: u64,
    gap: usize,
    ru: RuWatch,
    /// Whether this end began the renegotiation in hand.
    reneg_ours: bool,
    renegs_by_peer: u32,
    /// Renegotiations this end has begun, and seconds running of samples
    /// falling too far from their points.
    renegs: u32,
    poor: u32,
    /// Extra spacing asked of the next design, in dB.
    margin_db: f64,
    /// Which symbol of twelve, in the Listener's count, starts a data frame.
    frame_phase: usize,
    /// Sequences parsed in phase 4, and whether unparsed bits were shown.
    parsed: u32,
    dumped: bool,
    /// The CPu in force before a renegotiation took it.
    cpu_before: Option<UpCp>,
    /// V92_RENEG_KEEP_CPD=1: a rate renegotiation sends the CPd already in
    /// use rather than a new design (experiment: does a changed precoder and
    /// prefilter leave a Conexant unable to read us afterwards?).
    keep_design_in_reneg: bool,
    /// Experiment (V92_RENEG_KEEP_DOWN=1): after a renegotiation, keep
    /// sending downstream on the CPu from before it, to learn whether the
    /// analogue modem's receiver stays there.
    keep_down_in_reneg: bool,
    /// V92_RENEG_ED_AIM: in a renegotiation, Ed aimed to reach the analogue
    /// modem this many seconds after it sends E2u, rather than going out
    /// when E2u is heard (a round trip after it was sent).
    pub(super) reneg_ed_aim: Option<f64>,
    /// TRN2d's least length in a rate renegotiation, in seconds.
    reneg_trn2d: f64,
    /// When the first CPu of this Phase 4 was heard.
    cpu_at: Option<u64>,
    /// How long after its first CPu the analogue modem's E2u was heard, in
    /// seconds, as last measured on this call.
    cpu_to_e2u: Option<f64>,
    /// The silence and acknowledge bits of the last SUVu logged.
    suvu_seen: Option<(bool, bool)>,
    /// Downstream symbols still to be left out (see `slip_downstream`).
    slips: usize,
    /// The first TRN1u's residual, heard while this end was silent.
    quiet_noise: Option<f64>,
    /// The noise the design was made for, in codec units.
    design_sigma: f64,
    /// The design made from TRN2u in phase 4, while it is being worked out.
    design2_rx: Option<DesignSlot>,
    /// When phase 4 began.
    phase4_at: u64,
    law: Law,
    request: PcmUpstreamRequest,
    round_trip: f64,
    now: u64,
    stage: Stage,
    status: Status,
    deadline: Option<(u64, &'static str)>,
    source: Source,
    listener: Listener,
    heard: Bits,
    ja: Option<JaDescriptor>,
    /// Su's turns: the first to S-bar, back to Su, and the one answering Jp.
    turns: Vec<SuTurn>,
    su_listening: bool,
    jp_sent: bool,
    /// Samples received when Su came back from S̄u; and what Jp asked for.
    su_back: Option<usize>,
    jp_extension: u16,
    trn1d_at: Option<u64>,
    /// Where the channel estimate's leading near-zero taps were cut, and the
    /// design running or done.
    design_rx: Option<DesignSlot>,
    design: Option<(usize, Design)>,
    cpt: Option<UpCp>,
    cpt_end: Option<usize>,
    trn2u_from: Option<usize>,
    cpu: Option<UpCp>,
    /// Where the last SUVu, CPu or CPus read ended, in bits.
    last_end: Option<usize>,
    /// The analogue modem's acknowledgement, in SUVu, CPu or E2u.
    acknowledged: bool,
    cpd_at: Option<u64>,
    cpd_repeating: bool,
    /// Once both ends have acknowledged: when Ed goes without E2u.
    ed_by: Option<u64>,
    b1u_at: Option<usize>,
    candidates: Vec<Candidate>,
    /// Receivers warmed, unread, by the sample they warmed from: a read
    /// position whose receiver would warm on the same samples takes the
    /// correction from here (the two pair offsets).
    warmed: Vec<(usize, Receiver)>,
    winner: Option<Candidate>,
    /// Bits of B1u still to come out of the winner.
    b1u_left: usize,
    recent: VecDeque<f64>,
    /// Absolute index of `recent`'s first sample.
    recent_from: usize,
    samples: usize,
    received: Vec<bool>,
    retrain_watch: RetrainWatch,
    wants_retrain: bool,
    retrain_why: Option<&'static str>,
    trace: Vec<String>,
    upstream: u32,
}

impl Machine {
    pub(super) fn new(settings: &Settings, request: PcmUpstreamRequest) -> Self {
        let mut source = Source::new(settings.law, request.uinfo, settings.jd, settings.habits.trn1d);
        source.jd = UpJd { rates: settings.jd.rates, lookahead: settings.jd.lookahead }.to_bits();
        // MD in 35 ms steps, as INFO1a gives it.
        let md = usize::from(request.md_units) * 35 * FS as usize / 1000;
        let mut m = Self {
            law: settings.law,
            request,
            round_trip: settings.round_trip,
            now: 0,
            stage: Stage::ReadJa,
            status: Status::Running,
            deadline: None,
            source,
            listener: Listener::new(settings.law, md),
            heard: Bits::default(),
            ja: None,
            turns: Vec::new(),
            su_listening: false,
            jp_sent: false,
            su_back: None,
            jp_extension: SU_EXTENSION,
            trn1d_at: None,
            design_rx: None,
            quiet_noise: None,
            design_sigma: 0.0,
            design2_rx: None,
            phase4_at: 0,
            listening: true,
            fed: 0,
            gap: 0,
            ru: RuWatch::default(),
            reneg_ours: false,
            renegs_by_peer: 0,
            renegs: 0,
            poor: 0,
            margin_db: 0.0,
            frame_phase: 0,
            parsed: 0,
            dumped: false,
            cpu_before: None,
            slips: 0,
            keep_design_in_reneg: std::env::var("V92_RENEG_KEEP_CPD").is_ok_and(|v| v == "1"),
            keep_down_in_reneg: std::env::var("V92_RENEG_KEEP_DOWN").is_ok_and(|v| v == "1"),
            reneg_ed_aim: std::env::var("V92_RENEG_ED_AIM").ok().and_then(|v| v.parse::<f64>().ok()).filter(|v| v.is_finite()),
            cpu_at: None,
            cpu_to_e2u: None,
            suvu_seen: None,
            reneg_trn2d: std::env::var("V92_RENEG_TRN2D").ok().and_then(|v| v.parse::<f64>().ok()).filter(|v| v.is_finite() && *v >= 0.0).unwrap_or(RENEG_TRN2D_DEFAULT),
            design: None,
            cpt: None,
            cpt_end: None,
            trn2u_from: None,
            cpu: None,
            last_end: None,
            acknowledged: false,
            cpd_at: None,
            cpd_repeating: false,
            ed_by: None,
            b1u_at: None,
            candidates: Vec::new(),
            warmed: Vec::new(),
            winner: None,
            b1u_left: 0,
            recent: VecDeque::with_capacity(RECENT + 1),
            recent_from: 0,
            samples: 0,
            received: Vec::new(),
            retrain_watch: RetrainWatch::new(Role::Answer, FS),
            wants_retrain: false,
            retrain_why: None,
            trace: Vec::new(),
            upstream: 0,
        };
        // 9.5.1.2.1: Ja within 4500 ms plus a round trip of INFO1a. The
        // Listener needs Ru and 2040T of TRN1u first; MD adds to it.
        let md_seconds = md as f64 / FS;
        m.deadline = Some((m.at(4.5 + md_seconds + settings.round_trip), "no Ja within 4500 ms of INFO1a (V.92 9.5.1.2.1)"));
        m.say("V.92 PCM upstream: listening for Ru");
        m
    }

    fn at(&self, seconds: f64) -> u64 {
        self.now + (seconds * FS).round() as u64
    }

    fn say(&mut self, line: impl Into<String>) {
        let line = line.into();
        self.trace.push(format!("v92 pcm-up {:.3}s: {line}", self.now as f64 / FS));
    }

    fn retrain(&mut self, why: &'static str) {
        if !self.wants_retrain {
            self.say(format!("{why}: retrain"));
            self.retrain_why = Some(why);
            self.wants_retrain = true;
        }
    }

    fn fail(&mut self, why: &'static str) {
        self.say(format!("failed: {why}"));
        self.status = Status::Failed(why);
        self.stage = Stage::Finished;
        self.source.start(Out::Silence);
        self.source.pending = None;
    }

    pub(super) fn status(&self) -> Status {
        self.status
    }

    pub(super) fn phase(&self) -> &'static str {
        match self.stage {
            Stage::ReadJa => "V.92 phase 3: training (PCM upstream)",
            Stage::SendJd => "V.92 phase 3: Jd/Jp",
            Stage::ReadCpt | Stage::AwaitE1u => "V.92 phase 3: DIL",
            Stage::Phase4 | Stage::B1u => "V.92 phase 4 (PCM upstream)",
            Stage::Data => "V.92 data mode (PCM upstream)",
            Stage::Reneg => "V.92 rate renegotiation (PCM upstream)",
            Stage::Finished => "V.92 finished",
        }
    }

    pub(super) fn take_trace(&mut self) -> Vec<String> {
        std::mem::take(&mut self.trace)
    }

    pub(super) fn take_retrain(&mut self) -> bool {
        std::mem::take(&mut self.wants_retrain)
    }

    pub(super) fn start_retrain(&mut self) {
        self.retrain("the modem above asked for a retrain");
    }

    pub(super) fn retrain_why(&self) -> Option<&'static str> {
        self.retrain_why
    }

    pub(super) fn take_bits(&mut self) -> Vec<bool> {
        std::mem::take(&mut self.received)
    }

    pub(super) fn send_bits(&mut self, bits: &[bool]) {
        self.source.data.extend(bits.iter().copied());
    }

    pub(super) fn pending_bits(&self) -> usize {
        self.source.data.len()
    }

    pub(super) fn carrier(&self) -> bool {
        self.stage == Stage::Data
    }

    pub(super) fn descriptor(&self) -> Option<&Descriptor> {
        self.ja.as_ref().map(|j| &j.dil)
    }

    pub(super) fn phase3_snr(&self) -> Option<f64> {
        None
    }

    /// The analogue modem is silent from Ja's end until Su: TRN1d after a
    /// round trip, and Jd until Su arrives, is all echo.
    pub(super) fn echo_training_window(&self) -> bool {
        self.stage == Stage::SendJd
            && !self.su_listening_heard()
            && match self.source.out {
                Out::Trn1d => self.source.count > ((self.round_trip + 0.080) * FS) as usize,
                Out::Jd => true,
                _ => false,
            }
    }

    fn su_listening_heard(&self) -> bool {
        !self.turns.is_empty()
    }

    /// One network sample in (full scale 1.0), one out.
    pub(super) fn step(&mut self, input: f64) -> f64 {
        self.now += 1;
        let linear = input * 32768.0;
        self.samples += 1;
        self.recent.push_back(linear);
        if self.recent.len() > RECENT {
            self.recent.pop_front();
            self.recent_from += 1;
        }
        if self.stage != Stage::Finished && self.retrain_watch.feed(input, FS) {
            self.retrain("Tone A from the analogue modem (9.5.1.2)");
        }
        if !matches!(self.stage, Stage::Data | Stage::Finished) {
            if let Some((at, why)) = self.deadline
                && self.now > at
            {
                self.deadline = None;
                if let Some((from, strength, at)) = self.listener.search() {
                    self.say(format!(
                        "TRN1u not found: looked from sample {from}, {} are in, its signs lined up best at {at} with {strength:.2}",
                        self.listener.received()
                    ));
                }
                self.retrain(why);
            }
        }
        if !matches!(self.stage, Stage::B1u | Stage::Data | Stage::Finished) {
            if !self.listening && (self.samples as u64 - 1 - self.fed - self.gap as u64) % 12 == 0 {
                self.gap = (self.samples as u64 - 1 - self.fed) as usize;
                self.listening = true;
            }
            if self.listening {
                self.fed += 1;
                if let Some(Heard::Trained { snr_db }) = self.listener.push_linear(linear) {
                    self.trained(snr_db);
                }
            }
        }
        if matches!(self.stage, Stage::Data | Stage::Reneg)
            && let Some(event) = self.ru.feed(linear)
        {
            self.ru_event(event);
        }
        match self.stage {
            Stage::ReadJa => self.read_ja(),
            Stage::SendJd => self.send_jd(),
            Stage::ReadCpt => self.read_cpt(),
            Stage::AwaitE1u => self.await_e1u(),
            Stage::Phase4 => self.phase4(),
            Stage::B1u | Stage::Data => self.data(),
            Stage::Reneg => {}
            Stage::Finished => {}
        }
        if self.slips > 0 && self.stage == Stage::Data {
            self.slips -= 1;
            let _ = self.source.next();
        }
        let out = self.source.next();
        if let Some(sig) = self.source.changed.take() {
            // Jd' in V.90's names is Jp' here (8.6.4).
            let name = if sig == Out::JdPrime { "Jp'" } else { sig.name() };
            self.say(format!("going out: {name}"));
        }
        out
    }

    fn trained(&mut self, snr_db: f64) {
        let shape = self.listener.channel().map(|c| {
            let (at, peak) = c.h.iter().enumerate().fold((0, 0.0f64), |m, (i, v)| if v.abs() > m.1 { (i, v.abs()) } else { m });
            format!("main tap {peak:.0} at {at} of {}, residual rms {:.0}", c.h.len(), c.noise.sqrt())
        });
        self.say(format!("trained on TRN1u at {snr_db:.1} dB ({})", shape.unwrap_or_default()));
        if self.stage != Stage::ReadCpt {
            // The first TRN1u, this end silent: the line without our echo.
            self.quiet_noise = self.listener.channel().map(|c| c.noise);
            return;
        }
        // The second TRN1u: what the analogue modem's transmitter is designed
        // for (9.6.1.1.2's CPd).
        let Some(channel) = self.listener.channel().cloned() else { return };
        let base = self.listener.sample_of(0);
        let heard: Vec<f64> = self.listener.samples(base).iter().take(TRAIN).copied().collect();
        if heard.len() < TRAIN {
            return;
        }
        let law = self.law;
        let limits = Limits::from(&self.request);
        // Noise well over the first TRN1u's is our own echo left after
        // cancelling, with the codec's rounding of signal plus echo in it.
        let echo = echo_on_line(self.quiet_noise, channel.noise);
        if echo {
            self.say("designing for a line with our echo on it");
        }
        let signs = design::trn1u_signs(TRAIN);
        self.design_rx = Some(Self::design_job(channel, heard, signs, law, limits, echo, self.margin_db));
    }

    /// Design the analogue modem's transmitter for `channel`, off the sample
    /// path: about half a second of work. Tests run it in line so what they
    /// see does not hang on the scheduler.
    fn design_job(channel: design::Channel, heard: Vec<f64>, sent: Vec<f64>, law: Law, limits: Limits, echo: bool, margin_db: f64) -> DesignSlot {
        let slot: DesignSlot = Arc::new(Mutex::new(None));
        let out = slot.clone();
        let work = move || {
            // With echo the heard levels are not the codec's cells any
            // more, and the rounding is noise in its own right, larger where
            // the codec's steps are: it comes out of the measured figure
            // here and the design puts it back where each point lies.
            let measured = channel.noise.sqrt();
            // The estimate handed in is as short as the sample path can
            // afford to solve. Here there is time: one with taps before and
            // after it (V92_PCM_LONG=before,after) sees the line's long low
            // end, which the short one cuts off and the design then leaves
            // on the points as a smear.
            let mut note = String::new();
            let mut channel = channel;
            let mut shift = 0;
            let extra = std::env::var("V92_PCM_LONG").ok().and_then(|v| {
                let (a, b) = v.split_once(',')?;
                Some((a.trim().parse::<usize>().ok()?, b.trim().parse::<usize>().ok()?))
            });
            if let Some((before, after)) = extra {
                let taps = channel.h.len() + before + after;
                if before + after > 0 && before < heard.len() && heard.len().min(sent.len()) >= 12 * taps {
                    let long = design::estimate(&heard[..heard.len() - before], &sent[before..], taps);
                    note = format!("; {taps} taps leave {:.1} rms", long.noise.sqrt());
                    channel = long;
                    shift = before;
                }
            }
            let (heard, sent) = (&heard[..heard.len() - shift], &sent[shift..]);
            let channel = if echo { design::unrounded(&channel, heard, law) } else { design::analogue(&channel, heard, sent, law) };
            // The estimate starts a lead before the symbol; its first taps
            // are the line's delay, which the prefilter need not spend taps
            // on. Only taps under half a percent go: precursors a few
            // percent high are ISI the design has to see.
            let peak = channel.h.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            let cut = channel.h.iter().position(|v| v.abs() > 0.005 * peak).unwrap_or(0).saturating_sub(2).max(shift);
            let trimmed = design::Channel { h: channel.h[cut..].to_vec(), noise: channel.noise };
            let target = Target { law, power: 1.0, margin_db, limits, echo };
            let result = design::design(&trimmed, &target);
            if let Ok(mut s) = out.lock() {
                let sigma = if echo { measured } else { channel.noise.sqrt() };
                *s = Some((cut - shift, result, sigma, format!("{:.1} under the codec's rounding{note}", channel.noise.sqrt())));
            }
        };
        if cfg!(test) {
            work();
        } else {
            std::thread::spawn(work);
        }
        slot
    }

    fn read_ja(&mut self) {
        let more = self.listener.take_bits_with_symbols();
        self.heard.extend(more);
        let Some((ja, _, _)) = self.heard.next(JaDescriptor::from_bits, |j| j.to_bits().len()) else { return };
        self.say(format!("Ja heard: DIL of {} segments, upstream rates {:#x}", ja.dil.ucodes.len(), ja.upstream_rates));
        let d = &ja.dil;
        self.source.dil = d.symbols().collect();
        let mut end = 0;
        self.source.dil_ends = d
            .ucodes
            .iter()
            .map(|&u| {
                end += d.segment_length(u);
                end
            })
            .collect();
        self.ja = Some(ja);
        self.heard.clear();
        // 9.5.1.1.3: Sd 384T, S-bar-d 48T, TRN1d, Jd.
        self.source.change(Out::Sd);
        self.stage = Stage::SendJd;
        self.deadline = None;
    }

    fn send_jd(&mut self) {
        // Phase 3's two-point bits mean nothing while Su is sent.
        self.listener.take_bits();
        if self.source.out == Out::Trn1d && self.trn1d_at.is_none() {
            self.trn1d_at = Some(self.now);
            // 9.5.1.2.2: Su within 5100 ms plus a round trip of TRN1d, and
            // SU_GRACE on top: the Conexant's Su usually comes 4.9 s after
            // TRN1d and now and then 5.5 s, and Jd is still going out.
            self.deadline = Some((self.at(5.1 + SU_GRACE + self.round_trip), "no Su within 5100 ms and its grace of TRN1d (V.92 9.5.1.2.2)"));
        }
        if self.source.out == Out::Jd && !self.su_listening {
            // 9.5.1.1.4: Jd, and the receiver set for Su.
            self.su_listening = true;
            self.listener.listen_su();
        }
        for turn in self.listener.take_su_turns() {
            self.say(format!("Su turn: {turn:?}"));
            self.turns.push(turn);
        }
        let to_bar: Vec<usize> = self.turns.iter().filter_map(|t| match t { SuTurn::ToBar(n) => Some(*n), _ => None }).collect();
        let back = self.turns.iter().any(|t| matches!(t, SuTurn::ToSu(_)));
        if !self.jp_sent && back {
            // 9.5.1.1.7-8: Su's phase is measured once it has settled after
            // the half symbol, and Jp asks for what puts the analogue modem's
            // symbols on the codec's samples. Without that the second TRN1u
            // lands wherever the modem's clock left it, and at the worst
            // phases the transmitter designed for it does not carry data.
            let since = *self.su_back.get_or_insert(self.listener.received());
            if self.listener.received() >= since + SU_SETTLE {
                self.jp_sent = true;
                let measured = self.listener.su_phase(SU_MEASURE).zip(self.listener.best_phase());
                // V92_PCM_JP_EXT: a fixed extension, for finding what a
                // modem does with one.
                let forced: Option<f64> = std::env::var("V92_PCM_JP_EXT").ok().and_then(|v| v.parse().ok());
                let measured = match (measured, forced) {
                    (Some((now, _)), Some(f)) => Some((now, (now + f).rem_euclid(1.0))),
                    (m, _) => m,
                };
                self.jp_extension = match measured {
                    Some((now, best)) => {
                        // Kept off the ends of the range, where a rounding
                        // either way is a whole symbol; the peak is flat there.
                        let extension = (best - now).rem_euclid(1.0).clamp(0.03, 0.97);
                        self.say(format!(
                            "Su is {now:.2} of a symbol behind TRN1u's instants, the codec's best is {best:.2}: Jp asks for {extension:.2} after this Jd"
                        ));
                        ((extension * 65536.0).round() as u32 % 65536) as u16
                    }
                    None => {
                        self.say("Su back from S-bar-u: Jp after this Jd");
                        SU_EXTENSION
                    }
                };
                self.source.jd =
                    Jp { su_extension: self.jp_extension, eight_point_training: false, eight_point_renegotiation: false }.to_bits();
                self.deadline = Some((self.at(5.0 + 2.0 * self.round_trip), "no S-bar-u answering Jp"));
            }
        }
        if self.jp_sent && to_bar.len() >= 2 && self.source.pending.is_none() {
            // 9.5.1.1.9-11: Jp', then DIL or SCR, and TRN1u to be heard.
            let dil = self.ja.as_ref().is_some_and(|j| !j.dil.is_empty());
            self.source.after_jd_prime = if dil { Out::Dil } else { Out::Scr };
            self.source.change(Out::JdPrime);
            self.listener.expect_trn1u(to_bar[1] + 24);
            if let Some((from, _, _)) = self.listener.search() {
                self.say(format!("TRN1u looked for from sample {from}; {} are in", self.listener.received()));
            }
            self.heard.clear();
            self.stage = Stage::ReadCpt;
            // 9.5.2.1.11: TRN1u then CPt within 5000 ms of the S-bar-u.
            self.deadline = Some((self.at(5.5 + 2.0 * self.round_trip), "no CPt within 5000 ms of S-bar-u (V.92 9.5.2.1.11)"));
            self.say(format!("S-bar-u answers Jp: Jp' then {}", if dil { "DIL" } else { "SCR" }));
        }
    }

    fn poll_design(&mut self) {
        let Some(slot) = self.design_rx.clone() else { return };
        let taken = match slot.lock() {
            Ok(mut s) => s.take(),
            Err(_) => None,
        };
        match taken {
            Some((cut, Ok(d), sigma, floor)) => {
                self.design_sigma = sigma;
                self.say(format!("design on TRN1u: noise sigma {sigma:.1} ({floor}), drn {} ({} bit/s), delay {} (+{cut})", d.cpd.drn, phase4::upstream_rate(d.cpd.drn).unwrap_or(0), d.delay));
                self.design = Some((cut, d));
                self.design_rx = None;
            }
            Some((_, Err(e), sigma, _)) => {
                // Not the end: TRN2u is measured again in phase 4 (8.7.6).
                self.design_rx = None;
                self.say(format!("design on TRN1u failed: {e:?} (noise sigma {sigma:.1} in codec units)"));
            }
            None => {}
        }
    }

    fn read_cpt(&mut self) {
        self.poll_design();
        let dil = self.ja.as_ref().is_some_and(|j| !j.dil.is_empty());
        if !dil && self.listener.trained() && self.source.out == Out::Scr && self.source.pending.is_none() && self.cpt.is_none() {
            // 9.5.1.1.13: trained, so Ri; CPt is asked for by it.
            self.source.change(Out::Ri);
        }
        let more = self.listener.take_bits_with_symbols();
        self.heard.extend(more);
        let found = self.heard.next(UpInfo::from_bits, |i| match i {
            UpInfo::Cp(c) => c.to_bits().len(),
            _ => 1,
        });
        let Some((UpInfo::Cp(cpt), at, len)) = found else { return };
        if cpt.kind != CpKind::Training {
            return;
        }
        let Some(training) = Mapping::from_cp(&to_cp(&cpt)) else {
            self.fail("the analogue modem's CPt is not one this end can send");
            return;
        };
        self.say(format!("CPt heard: drn {} ({} bit/s training)", cpt.drn, sequences::training_bits(cpt.drn) * 8000 / 6));
        self.source.training = Some(training);
        // 9.5.1.1.12: Ri on CPt; with no DIL, R-bar-i (9.5.1.1.13).
        self.source.change(if dil { Out::Ri } else { Out::RiBar });
        self.cpt = Some(cpt);
        self.cpt_end = Some(at + len);
        self.stage = Stage::AwaitE1u;
    }

    fn await_e1u(&mut self) {
        self.poll_design();
        let more = self.listener.take_bits_with_symbols();
        self.heard.extend(more);
        let Some(end) = self.cpt_end else { return };
        // Another CPt, or zeros: the fill, any padding, and E1u's 12.
        let (zeros, ended) = self.heard.zeros_from(end);
        if ended && zeros < 12 {
            // Not E1u: look again from the next CPt.
            let found = self.heard.next(UpInfo::from_bits, |i| match i {
                UpInfo::Cp(c) => c.to_bits().len(),
                _ => 1,
            });
            if let Some((UpInfo::Cp(c), at, len)) = found
                && c.kind == CpKind::Training
            {
                self.cpt_end = Some(at + len);
            }
            return;
        }
        if zeros < 12 {
            return;
        }
        let Some(first) = self.heard.symbol(end) else { return };
        // TRN2u starts on a frame, after the E1u that follows the last CPt.
        let from = ceil12(first + 12);
        self.listener.switch_trn2u(from, false);
        self.heard.drop_from(from);
        self.heard.scan = self.heard.bits.len();
        self.trn2u_from = Some(from);
        let dil = self.ja.as_ref().is_some_and(|j| !j.dil.is_empty());
        if dil {
            self.source.change(Out::RiBar);
        }
        self.source.up_info = Some(Suvd { silence: false, acknowledge: false }.to_bits());
        // 9.6.1.1.1: SUVd when ready for CPu, which is once TRN2u is measured.
        self.source.hold_trn2d = true;
        self.phase4_at = self.now;
        self.ed_by = None;
        self.stage = Stage::Phase4;
        // 9.6.1.2.1: B1u within 20 s plus 6 round trips of INFO1a; the
        // start-up so far is most of that.
        self.deadline = Some((self.at(12.0 + 6.0 * self.round_trip), "no B1u (V.92 9.6.1.2.1)"));
        self.say(format!("E1u heard: R-bar-i, phase 4; TRN2u from symbol {from}"));
    }

    fn cpd_bits(&self) -> Option<Vec<bool>> {
        let (_, d) = self.design.as_ref()?;
        Cpd { acknowledge: self.cpu.is_some(), ..d.cpd.clone() }.to_bits()
    }

    fn up_info_now(&mut self) {
        let ack = self.cpu.is_some();
        self.source.mp_ack = ack;
        self.source.up_info = if self.cpd_repeating {
            self.cpd_bits()
        } else {
            Some(Suvd { silence: false, acknowledge: ack }.to_bits())
        };
    }

    /// Leave one downstream symbol out, in data mode: the analogue modem's
    /// data frames then begin one symbol sooner against ours. A path that
    /// dropped or added samples (a jitter buffer's 160, say: not a multiple
    /// of 6) leaves the modem reading frames at the wrong boundary for good
    /// -- a rate renegotiation keeps that alignment (9.8) -- and at most five
    /// of these find the right one again without a retrain. Every sample
    /// still goes out at its time; only which symbol it carries moves.
    pub(super) fn slip_downstream(&mut self) -> bool {
        if self.stage != Stage::Data {
            return false;
        }
        self.slips += 1;
        self.say("downstream slipped by one symbol");
        true
    }

    /// Begin a rate renegotiation from data mode (9.8.1.1): Rd, R-bar-d and
    /// TRN2d go out, and Ru is waited for. False outside data mode.
    pub(super) fn renegotiate(&mut self, why: &str) -> bool {
        if self.stage != Stage::Data {
            return false;
        }
        self.renegs += 1;
        self.say(format!("rate renegotiation begun by this end: {why}; margin {:.1} dB", self.margin_db));
        self.enter_reneg(true);
        self.stage == Stage::Reneg
    }

    /// The last rate renegotiation was begun by the analogue modem.
    pub(super) fn renegotiated_by_peer(&self) -> bool {
        self.renegs_by_peer > 0 && !self.reneg_ours
    }

    fn enter_reneg(&mut self, ours: bool) {
        let (Some(cpt), Some(cpu)) = (self.cpt.as_ref(), self.cpu.as_ref()) else { return };
        let cp = to_cp(cpu);
        let Some(training) = Mapping::for_renegotiation(&to_cp(cpt), &cp) else {
            self.retrain("the renegotiation has no mapping to train on");
            return;
        };
        self.source.r_codes = std::array::from_fn(|i| cp.points(i).last().copied().unwrap_or(0));
        self.source.training = Some(training);
        self.source.mp_ack = false;
        self.source.sending_acknowledged = false;
        self.source.acknowledged = 0;
        self.source.mps_sent = 0;
        self.source.up_once = None;
        self.source.up_info = Some(Suvd { silence: false, acknowledge: false }.to_bits());
        // TRN2d until TRN2u is measured and the new CPd designed, and for
        // no less than reneg_trn2d whichever end began it.
        self.source.hold_trn2d = true;
        // Six symbols a frame, a symbol a sample.
        self.source.trn2d_min = ((self.reneg_trn2d * FS / 6.0).ceil() as usize)
            .clamp(super::digital::TRN2D_FRAMES, super::digital::TRN2D_MAX_FRAMES);
        if ours {
            self.source.change(Out::Rd);
            self.ru = RuWatch::default();
        }
        self.reneg_ours = ours;
        if !ours {
            self.renegs_by_peer += 1;
        }
        self.poor = 0;
        self.winner = None;
        self.candidates.clear();
        self.status = Status::Running;
        self.stage = Stage::Reneg;
        self.deadline = Some((self.at(6.0 + 2.0 * self.round_trip), "no Ru-to-R-bar-u in the rate renegotiation (V.92 9.8)"));
    }

    fn ru_event(&mut self, event: RuEvent) {
        match (event, self.stage) {
            (RuEvent::Heard, Stage::Data) => {
                // 9.8.1.2.1: the analogue modem's Ru; circuit 104 clamped.
                self.say("Ru heard in data mode: rate renegotiation begun by the analogue modem");
                self.enter_reneg(false);
            }
            (RuEvent::Reversal, Stage::Reneg) if self.listening => {
                if !self.reneg_ours {
                    // 9.8.1.2.2: Rd for 384T, R-bar-d for 24T, then TRN2d.
                    self.source.change(Out::Rd);
                }
                // The first R-bar-u symbol, in the Listener's count: its
                // response peaks at the channel's main tap. TRN2u follows 24
                // symbols on, at the start of a data frame.
                let main = self.listener.channel().map_or(0, |c| {
                    c.h.iter().enumerate().fold((0, 0.0f64), |m, (i, v)| if v.abs() > m.1 { (i, v.abs()) } else { m }).0
                });
                let here = (self.fed as usize).saturating_sub(1 + self.listener.sample_of(0) + main + 4);
                let phase = self.frame_phase;
                let from = (here + 24 + 6 - phase) / 12 * 12 + phase;
                self.listener.take_bits_with_symbols();
                self.listener.switch_trn2u(from, false);
                self.heard = Bits::default();
                self.trn2u_from = Some(from);
                self.cpu_before = self.cpu.take();
                self.cpu_at = None;
                self.suvu_seen = None;
                self.cpd_at = None;
                self.cpd_repeating = false;
                self.acknowledged = false;
                self.ed_by = None;
                self.b1u_at = None;
                self.last_end = None;
                self.design2_rx = None;
                self.phase4_at = self.now;
                self.ru = RuWatch::default();
                self.stage = Stage::Phase4;
                self.deadline = Some((self.at(12.0 + 6.0 * self.round_trip), "no B1u in the rate renegotiation (V.92 9.6.1.2.1)"));
                self.say(format!("Ru-to-R-bar-u: TRN2u from symbol {from}"));
            }
            _ => {}
        }
    }

    /// 8.7.6: "TRN2u may be used to estimate the analogue channel for
    /// upstream". It is heard with TRN2d going out and the echo canceller
    /// settled, which is how data will be heard; the second TRN1u was heard
    /// across Jp, Jp' and the DIL's start.
    fn measure_trn2u(&mut self) {
        let Some(from) = self.trn2u_from else { return };
        if !self.source.hold_trn2d {
            return;
        }
        if self.design2_rx.is_none() {
            let first = from + TRN2U_SKIP;
            let sent: Vec<f64> = self.listener.decided(first).iter().take(TRN2U_MEASURE).copied().collect();
            let late = self.now > self.phase4_at + (TRN2U_WAIT * FS) as u64;
            if sent.len() < TRN2U_MEASURE {
                if late && self.design.is_some() {
                    self.say("TRN2u not measured in time: the TRN1u design stands");
                    self.source.hold_trn2d = false;
                }
                return;
            }
            let heard: Vec<f64> = self.listener.samples(self.listener.sample_of(first)).iter().take(TRN2U_MEASURE).copied().collect();
            if heard.len() < TRN2U_MEASURE {
                return;
            }
            let taps = self.listener.channel().map_or(80, |c| c.h.len());
            let channel = design::estimate(&heard, &sent, taps);
            let (at, peak) = channel.h.iter().enumerate().fold((0, 0.0f64), |m, (i, v)| if v.abs() > m.1 { (i, v.abs()) } else { m });
            self.say(format!("TRN2u measured: main tap {peak:.0} at {at} of {taps}, residual rms {:.1}", channel.noise.sqrt()));
            let echo = echo_on_line(self.quiet_noise, channel.noise);
            if echo {
                self.say("designing for a line with our echo on it");
            }
            self.design2_rx = Some(Self::design_job(channel, heard, sent, self.law, Limits::from(&self.request), echo, self.margin_db));
        }
        let taken = match self.design2_rx.as_ref().map(|s| s.lock()) {
            Some(Ok(mut s)) => s.take(),
            _ => None,
        };
        match taken {
            Some((_, Ok(d), sigma, _)) if self.keep_design_in_reneg && self.cpu_before.is_some() && self.design.is_some() => {
                self.say(format!("design on TRN2u: noise sigma {sigma:.1}, drn {} -- not used: the design in use stands (V92_RENEG_KEEP_CPD)", d.cpd.drn));
                self.source.hold_trn2d = false;
            }
            Some((cut, Ok(d), sigma, floor)) => {
                self.design_sigma = sigma;
                self.say(format!("design on TRN2u: noise sigma {sigma:.1} ({floor}), drn {} ({} bit/s), delay {} (+{cut})", d.cpd.drn, phase4::upstream_rate(d.cpd.drn).unwrap_or(0), d.delay));
                self.design = Some((cut, d));
                self.source.hold_trn2d = false;
            }
            Some((_, Err(e), sigma, _)) => {
                self.say(format!("design on TRN2u failed: {e:?} (noise sigma {sigma:.1} in codec units)"));
                if self.design.is_some() {
                    self.say("the TRN1u design stands");
                    self.source.hold_trn2d = false;
                } else {
                    self.fail("no PCM upstream design fits this line");
                }
            }
            None => {}
        }
    }

    /// Once a second in phase 4: how much has been read and where the
    /// reading stands, and once, the bits a frame sync was followed by when
    /// nothing could be made of them.
    fn phase4_report(&mut self) {
        if self.now % 8000 != 0 {
            return;
        }
        let plain = self.heard.plain(0);
        let ones = sequences::SYNC_ONES;
        let syncs = (0..plain.len().saturating_sub(ones + 1))
            .filter(|&i| (i == 0 || !plain[i - 1]) && plain[i..i + ones].iter().all(|&b| b) && !plain[i + ones])
            .count();
        let waiting = plain.len() - self.heard.scan.min(plain.len());
        self.say(format!(
            "phase 4: {} bits read, {syncs} frame syncs, {} sequences parsed, {waiting} bits not yet made sense of, CPu {}",
            plain.len(),
            self.parsed,
            if self.cpu.is_some() { "heard" } else { "not heard" }
        ));
        if waiting > 600 && !self.dumped {
            self.dumped = true;
            let from = self.heard.scan.min(plain.len());
            let bits: String = plain[from..].iter().take(1100).map(|&b| if b { '1' } else { '0' }).collect();
            self.say(format!("phase 4: unparsed from bit {from}: {bits}"));
        }
    }

    fn phase4(&mut self) {
        self.poll_design();
        self.measure_trn2u();
        if self.stage != Stage::Phase4 {
            return;
        }
        self.phase4_report();
        let more = self.listener.take_bits_with_symbols();
        self.heard.extend(more);
        while let Some((info, at, len)) = self.heard.next(UpInfo::from_bits, |i| match i {
            UpInfo::Cp(c) => c.to_bits().len(),
            UpInfo::Cpus(c) => c.to_bits().len(),
            UpInfo::Suv(s) => s.to_bits().len(),
        }) {
            match info {
                UpInfo::Suv(s) => {
                    if self.suvu_seen != Some((s.silence, s.acknowledge)) {
                        self.suvu_seen = Some((s.silence, s.acknowledge));
                        self.say(format!("SUVu: silence asked {}, acknowledge {}, wait for CPu {}, level {}", s.silence, s.acknowledge, s.wait_for_cpu, s.level_q2_2));
                    }
                    if s.acknowledge {
                        self.acknowledged = true;
                    }
                    if self.cpd_at.is_none() && self.design.is_some() && !self.source.hold_trn2d {
                        // 9.6.1.1.2: one CPd after SUVu.
                        self.source.up_once = self.cpd_bits();
                        self.cpd_at = Some(self.now);
                        self.say("SUVu heard: CPd going out");
                        // The CPd as sent, for reading B1u offline against
                        // what the analogue modem was told to send.
                        if let Some(bits) = self.source.up_once.clone() {
                            let hex: String = bits.chunks(4).map(|n| format!("{:x}", n.iter().fold(0u8, |v, &b| v << 1 | u8::from(b)))).collect();
                            self.say(format!("CPd bits ({}): {hex}", bits.len()));
                        }
                    }
                }
                UpInfo::Cp(c) if c.kind == CpKind::Data => {
                    if c.drn == 0 {
                        self.fail("the analogue modem cleared down in phase 4");
                        return;
                    }
                    if self.cpu.as_ref().map(|p| p.to_bits()) != Some(c.to_bits()) {
                        // Whole, for reading the downstream back off a
                        // recording with the mapping the modem asked for.
                        let bits = c.to_bits();
                        let hex: String = bits.chunks(4).map(|n| format!("{:x}", n.iter().fold(0u8, |v, &b| v << 1 | u8::from(b)))).collect();
                        self.say(format!("CPu bits ({}): {hex}", bits.len()));
                    }
                    let Some(data_mode) = Mapping::from_cp(&to_cp(&c)) else {
                        self.fail("the analogue modem's CPu is not one this end can send");
                        return;
                    };
                    if c.acknowledge {
                        self.acknowledged = true;
                    }
                    if self.cpu.is_none() {
                        self.cpu_at.get_or_insert(self.now);
                        self.say(format!("CPu heard: drn {} ({} bit/s down), ack {}", c.drn, sequences::data_rate(c.drn).unwrap_or(0), c.acknowledge));
                    }
                    let data_mode = match self.cpu_before.as_ref().filter(|_| self.keep_down_in_reneg).and_then(|b| Mapping::from_cp(&to_cp(b))) {
                        Some(old) => {
                            if self.cpu.is_none() {
                                self.say(format!("experiment: downstream stays on the CPu from before the renegotiation ({} bit/s)", old.rate()));
                            }
                            old
                        }
                        None => data_mode,
                    };
                    self.source.data_mode = Some(data_mode);
                    self.cpu = Some(c);
                    self.up_info_now();
                }
                UpInfo::Cpus(c) => {
                    if c.acknowledge {
                        self.acknowledged = true;
                    }
                }
                UpInfo::Cp(_) => {}
            }
            self.last_end = Some(at + len);
            self.parsed += 1;
        }
        // E2u: a data frame of zeros after the last sequence, past any
        // padding (fewer than 12 symbols of it), with B1u's bits after it.
        if let Some(end) = self.last_end
            && self.cpu.is_some()
            && self.b1u_at.is_none()
        {
            let (zeros, ended) = self.heard.zeros_from(end);
            if zeros >= 24 && ended {
                let first = self.heard.symbol(end).unwrap_or(0);
                self.acknowledged = true;
                self.b1u_at = Some(first + 12);
                self.say(format!("E2u heard from symbol {first}"));
                if let Some(at) = self.cpu_at {
                    let gap = (self.now - at) as f64 / FS;
                    self.cpu_to_e2u = Some(gap);
                    self.say(format!("E2u heard {gap:.3} s after the first CPu"));
                }
            }
        }
        // 9.6.1.1.3: no acknowledgement a round trip and 100 ms after CPd.
        if !self.cpd_repeating
            && !self.acknowledged
            && self.cpd_at.is_some_and(|at| self.now > at + ((0.4 + self.round_trip) * FS) as u64)
        {
            self.cpd_repeating = true;
            self.up_info_now();
            self.say("no acknowledgement of CPd: CPd repeated");
            if let Some(bits) = self.cpd_bits() {
                let hex: String = bits.chunks(4).map(|n| format!("{:x}", n.iter().fold(0u8, |v, &b| v << 1 | u8::from(b)))).collect();
                self.say(format!("CPd bits ({}): {hex}", bits.len()));
            }
        }
        // A renegotiation's Ed aimed at the analogue modem's E2u: sent so as
        // to reach it `aim` seconds after it sends E2u, which it does a
        // steady time after its first CPu (0.76 s from a CX93001), so that
        // Ed does not land a whole round trip late.
        if let Some(aim) = self.reneg_ed_aim
            && self.cpu_before.is_some()
            && self.source.out == Out::Mp
            && self.source.data_mode.is_some()
            && self.source.pending.is_none()
            && self.b1u_at.is_none()
            && let Some(at) = self.cpu_at
        {
            let gap = self.cpu_to_e2u.unwrap_or(0.762);
            let wait = (gap + aim - self.round_trip).max(0.0);
            if self.now >= at + (wait * FS) as u64 {
                self.say(format!("aimed Ed: {wait:.3} s after the first CPu (E2u expected {gap:.3} s after it, round trip {:.3} s)", self.round_trip));
                self.source.change(Out::Ed);
            }
        }
        // 9.6.1.1.4: Ed once both ends have acknowledged.
        if self.source.out == Out::Mp
            && self.source.acknowledged >= 1
            && self.acknowledged
            && self.source.data_mode.is_some()
            && self.source.pending.is_none()
        {
            // Ed after E2u, the order a real Conexant takes in start-up and
            // is heard in every time. Ed on the acknowledgements alone (the
            // SUVu' a renegotiation brings first) left that modem deaf to
            // our data after 30 of 35 renegotiations on 2026-11-13 -- its
            // E2u was still to come. 9.6.1.1.4 allows either; a modem that
            // waits for Ed (9.6.2.1.4) gets it a round trip and 400 ms on.
            let by = *self.ed_by.get_or_insert(self.now + ((0.4 + self.round_trip) * FS) as u64);
            if self.b1u_at.is_some() {
                self.say("acknowledged both ways and E2u heard: Ed, B1d");
                self.source.change(Out::Ed);
            } else if self.now >= by {
                self.say("acknowledged both ways, no E2u a round trip on: Ed, B1d");
                self.source.change(Out::Ed);
            }
        }
        if let Some(b1u) = self.b1u_at {
            self.start_b1u(b1u);
        }
    }

    fn start_b1u(&mut self, b1u: usize) {
        let Some((cut, design)) = self.design.clone() else {
            self.fail("E2u came before the design was done");
            return;
        };
        let Ok(params) = UpParams::from_cpd(&design.cpd) else {
            self.fail("the designed CPd is not one the receiver takes");
            return;
        };
        let base = self.listener.sample_of(0);
        let mut starts: Vec<usize> = vec![ceil12(b1u), b1u];
        starts.dedup();
        self.candidates.clear();
        self.warmed.clear();
        // 0: Ki as 6.4.2 prints it (USR, Conexant). 2: BinModem 4, whose
        // pair-opening indices sit two above (its PAIR_FIRST_INDEX_OFFSET).
        // B1u's ones say which this caller is.
        for pair_offset in [0, 2] {
        for &start in &starts {
            for offset in -2i64..=2 {
                let Ok(rx) = Receiver::new(params.clone(), self.law, Decoding::Viterbi) else {
                    self.fail("the PCM upstream receiver refused the design");
                    return;
                };
                let rx = rx.with_pair_offset(pair_offset);
                // Learn and take out what the design left smeared across
                // neighbouring samples (V92_PCM_CORRECT=0 turns it off).
                let rx = if std::env::var("V92_PCM_CORRECT").is_ok_and(|v| v == "0") { rx } else { rx.corrected() };
                let next = (base + start + cut + design.delay + self.gap) as i64 + offset;
                if next < self.recent_from as i64 {
                    continue;
                }
                self.candidates.push(Candidate { rx, start, next: next as usize, out: Vec::new(), warmed: false });
            }
        }
        }
        self.b1u_left = B1U_FRAMES * params.k as usize;
        self.upstream = phase4::upstream_rate(design.cpd.drn).unwrap_or(0);
        self.stage = Stage::B1u;
        self.listening = false;
        self.say(format!("B1u from symbol {b1u}: {} read positions tried", self.candidates.len()));
        // What the analogue modem was told to make B1u with, for reading it
        // offline (the design's CPd; CPd' differs only in bit 33).
        if let Some(bits) = design.cpd.to_bits() {
            let hex: String = bits.chunks(4).map(|n| format!("{:x}", n.iter().fold(0u8, |v, &b| v << 1 | u8::from(b)))).collect();
            self.say(format!("B1u expected under CPd bits ({}): {hex}", bits.len()));
        }
    }

    fn feed(recent: &VecDeque<f64>, from: usize, samples: usize, c: &mut Candidate, warmed: &mut Vec<(usize, Receiver)>) {
        if !c.warmed {
            // Nothing is read until the correction has been learnt from the
            // start of what is to be read: a quarter of a second's wait.
            if c.next >= from {
                if c.next + Receiver::WARM > samples {
                    return;
                }
                if !warmed.iter().any(|(at, rx)| *at == c.next && c.rx.warm_from(rx)) {
                    let first: Vec<f64> = recent.range(c.next - from..c.next - from + Receiver::WARM).copied().collect();
                    c.rx.warm(&first);
                    warmed.push((c.next, c.rx.clone()));
                }
            }
            c.warmed = true;
        }
        // The sample that came and CATCH_UP more of what is behind.
        let mut budget = CATCH_UP + 1;
        while c.next < samples && budget > 0 {
            budget -= 1;
            let Some(&v) = c.next.checked_sub(from).and_then(|i| recent.get(i)) else {
                c.next += 1;
                continue;
            };
            c.out.extend(c.rx.push_linear(v));
            c.next += 1;
        }
    }

    fn data(&mut self) {
        for c in self.candidates.iter_mut() {
            Self::feed(&self.recent, self.recent_from, self.samples, c, &mut self.warmed);
        }
        if let Some(w) = self.winner.as_mut() {
            Self::feed(&self.recent, self.recent_from, self.samples, w, &mut self.warmed);
            let mut out = std::mem::take(&mut w.out);
            let skip = self.b1u_left.min(out.len());
            self.b1u_left -= skip;
            out.drain(..skip);
            self.received.extend(out);
            // A diagnostic: the samples and their nearest points, to a file.
            if self.now % 8000 == 0
                && let Ok(dir) = std::env::var("V92_PCM_DUMP")
            {
                use std::io::Write;
                w.rx.trace();
                let bytes: Vec<u8> =
                    w.rx.take_trace().iter().flatten().flat_map(|v| v.to_le_bytes()).collect();
                let path = format!("{dir}/pcmup3-{}.f32", std::process::id());
                if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                    let _ = f.write_all(&bytes);
                }
            }
            // Once a second: how far the samples fall from the points.
            if self.now % 8000 == 0
                && let Some((miss, _)) = w.rx.take_miss()
            {
                let sigma = self.design_sigma;
                self.say(format!("data: samples fall {miss:.1} rms from their points (designed for a sigma of {sigma:.1})"));
                // Well past what the points were spaced for, five seconds
                // running: ask for points further apart (9.8.1.1).
                self.poor = if miss > POOR * sigma { self.poor + 1 } else { 0 };
                if self.poor >= 5 && self.renegs < MAX_RENEGS {
                    self.margin_db += RENEG_MARGIN_DB;
                    self.renegotiate("samples fall too far from their points");
                }
            }
            return;
        }
        // Each has read B1u through its decoder's depth once it has 20
        // frames of output: the one that reads ones is in step.
        let k = self.b1u_left / B1U_FRAMES;
        if self.candidates.is_empty() || self.candidates.iter().any(|c| c.out.len() < 20 * k) {
            return;
        }
        let score = |c: &Candidate| {
            // Only B1u's own frames: a receiver that waited to learn its
            // correction comes here with data behind them already.
            let tail = &c.out[k.min(c.out.len())..(20 * k).min(c.out.len())];
            tail.iter().filter(|&&b| b).count() as f64 / tail.len().max(1) as f64
        };
        let (best, s) = self
            .candidates
            .iter()
            .enumerate()
            .map(|(i, c)| (i, score(c)))
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap_or((0, 0.0));
        let scores: Vec<String> = self.candidates.iter().map(|c| format!("{:.2}", score(c))).collect();
        self.say(format!("B1u ones by read position: {}", scores.join(" ")));
        if s < 0.95 {
            self.candidates.clear();
            self.retrain("B1u did not read as ones at any position");
            return;
        }
        let w = self.candidates.swap_remove(best);
        self.frame_phase = w.start % 12;
        self.candidates.clear();
        self.warmed.clear();
        self.winner = Some(w);
        let downstream = self.cpu.as_ref().and_then(|c| sequences::data_rate(c.drn)).unwrap_or(0);
        self.status = Status::Connected { downstream, upstream: self.upstream };
        self.stage = Stage::Data;
        self.deadline = None;
        let offset = self.winner.as_ref().map_or(0, |w| w.rx.pair_offset());
        self.say(format!("B1u read: data mode, {downstream} down, {} up (PCM), pair offset {offset}", self.upstream));
    }
}

impl std::fmt::Debug for Machine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("pcmup::Machine").field("stage", &self.stage).field("status", &self.status).finish()
    }
}

#[cfg(test)]
mod tests {
    //! A scripted analogue modem answering what the Machine sends, by
    //! watching the Machine's own transmitter rather than demodulating it,
    //! through a telephone band at fractional symbol times and a codec.
    use super::*;
    use crate::v34::info::SymbolRate;
    use crate::v34::qam::Band;
    use crate::v90::digital::Habits;
    use crate::v90::sequences::Jd;
    use crate::v90::ucode;
    use crate::v92::phase4::{pad, Suvu};
    use crate::v92::upstream::training::{ru, su, Sender};
    use crate::v92::upstream::transmitter::Transmitter;
    use crate::v32::{Mode, Scrambler};
    use crate::v90::encoder::{Decoder, Frame};
    use crate::v90::INTERVALS;

    fn band(t: f64) -> f64 {
        if !(0.0..=30.0).contains(&t) {
            return 0.0;
        }
        let fs = 8000.0;
        let m = t - 15.0;
        let sinc = |x: f64| if x.abs() < 1e-12 { 1.0 } else { (std::f64::consts::PI * x).sin() / (std::f64::consts::PI * x) };
        let hamming = 0.54 - 0.46 * (std::f64::consts::TAU * t / 30.0).cos();
        (2.0 * 3400.0 / fs * sinc(2.0 * 3400.0 * m / fs) - 2.0 * 300.0 / fs * sinc(2.0 * 300.0 * m / fs)) * hamming
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    enum St {
        Start,
        Ja,
        Quiet,
        Su,
        Cpt,
        Trn2u,
        Suvu,
        Ack,
        B1u,
        Data,
    }

    enum Sym {
        V(f64),
        Wait(f64),
    }

    struct Analogue {
        law: Law,
        at: f64,
        queue: VecDeque<Sym>,
        st: St,
        s: Sender,
        sent: Vec<(f64, f64)>,
        rng: u64,
        since: u64,
        first: bool,
        tx: Option<Transmitter>,
        payload: Vec<bool>,
        dil: Descriptor,
        /// Rate renegotiations still to take part in, whether it begins
        /// them itself, and how much payload had gone before the last.
        renegs_left: u32,
        starts: bool,
        mark: usize,
        /// Samples of Jd it lets go by before its Su.
        slow_su: u64,
        jd_seen: u64,
        late_cpu: bool,
        /// E2u once this end's SUVd' or CPd' is in (9.6.2.1.4's first way), as
        /// a real Conexant does in start-up, rather than waiting for Ed.
        e2u_first: bool,
    }

    fn padded(mut bits: Vec<bool>, multiple: usize) -> Vec<bool> {
        pad(&mut bits, multiple);
        bits
    }

    fn suvu(ack: bool) -> Vec<bool> {
        padded(Suvu { wait_for_cpu: false, level_q2_2: -16, silence: false, acknowledge: ack }.to_bits(), 24)
    }

    /// A real modem asks a different downstream rate after a
    /// renegotiation (45333 then 44000 on the 2026-11-13 calls); this one
    /// asks drn 16 the first time and drn 14 after.
    fn cpu(ack: bool, renegotiated: bool) -> UpCp {
        let drn = if renegotiated { 14 } else { 16 };
        UpCp { kind: CpKind::Data, drn, acknowledge: ack, constellations: vec![(u128::MAX >> 1) & !1], ..UpCp::default() }
    }

    fn cpt() -> UpCp {
        UpCp { kind: CpKind::Training, drn: 9, constellations: vec![(40..104).step_by(4).fold(0u128, |m, u| m | 1 << u)], ..UpCp::default() }
    }

    impl Analogue {
        fn values(&mut self, v: Vec<f64>) {
            self.queue.extend(v.into_iter().map(Sym::V));
        }

        /// The script: what comes next, from what the Machine is doing.
        fn refill(&mut self, m: &Machine) {
            self.since += 1;
            match self.st {
                St::Start => {
                    self.values(vec![0.0; 400]);
                    self.values(ru(384, false));
                    self.values(ru(24, true));
                    let t = self.s.trn1u(2400);
                    self.values(t);
                    self.st = St::Ja;
                    self.first = true;
                }
                St::Ja => {
                    if m.stage != Stage::ReadJa && m.source.out != Out::Silence {
                        self.st = St::Quiet;
                        self.since = 0;
                        return self.refill(m);
                    }
                    let ja = JaDescriptor { dil: self.dil.clone(), upstream_rates: 0x7_ffff };
                    let mut bits = if self.first { vec![true; 24] } else { Vec::new() };
                    self.first = false;
                    bits.extend(ja.to_bits());
                    let v = self.s.two_point(&bits);
                    self.values(v);
                }
                St::Quiet => {
                    if m.source.out == Out::Jd {
                        self.jd_seen += 1;
                    }
                    if m.source.out == Out::Jd && self.since > 800 && self.jd_seen > self.slow_su {
                        self.values(su(144, false));
                        self.values(su(24, true));
                        self.queue.push_back(Sym::Wait(0.5));
                        self.st = St::Su;
                        self.since = 0;
                    } else {
                        self.values(vec![0.0]);
                    }
                }
                St::Su => {
                    if m.jp_sent && self.since > 3000 {
                        self.values(su(24, true));
                        self.queue.push_back(Sym::Wait(f64::from(m.jp_extension) / 65536.0));
                        let t = self.s.trn1u(3000);
                        self.values(t);
                        self.st = St::Cpt;
                        self.first = true;
                    } else {
                        self.values(su(6, false));
                    }
                }
                St::Cpt if !self.dil.is_empty() => {
                    // 9.5.2.1.11: CPt after the TRN1u until Ri, then E1u.
                    if m.cpt.is_some() && matches!(m.source.out, Out::Ri | Out::RiBar) {
                        let v = self.s.two_point(&[false; 12]);
                        self.values(v);
                        self.st = St::Trn2u;
                        return;
                    }
                    let mut bits = if self.first { vec![true; 24] } else { Vec::new() };
                    self.first = false;
                    bits.extend(padded(cpt().to_bits(), 12));
                    let v = self.s.two_point(&bits);
                    self.values(v);
                }
                St::Cpt => {
                    // No DIL asked for: CPt once Ri comes, E1u on R-bar-i
                    // (9.5.2.1.10).
                    if m.cpt.is_some() && m.source.out != Out::Ri {
                        let v = self.s.two_point(&[false; 12]);
                        self.values(v);
                        self.st = St::Trn2u;
                        return;
                    }
                    if m.source.out != Out::Ri {
                        let v = self.s.two_point(&[true; 12]);
                        self.values(v);
                        return;
                    }
                    let mut bits = if self.first { vec![true; 24] } else { Vec::new() };
                    self.first = false;
                    bits.extend(padded(cpt().to_bits(), 12));
                    let v = self.s.two_point(&bits);
                    self.values(v);
                }
                St::Trn2u => {
                    let v = self.s.trn2u(2400, false);
                    self.values(v);
                    self.st = St::Suvu;
                    self.since = 0;
                }
                St::Suvu => {
                    if m.cpd_at.is_some() && self.since > 400 {
                        // A CPu whose first copy was lost: SUVu' (CPd is in)
                        // reaches the digital modem first, as it did from a
                        // real Conexant in renegotiations on 2026-11-13.
                        let mut bits = if self.late_cpu && self.mark > 0 { [suvu(true), suvu(true), suvu(true)].concat() } else { Vec::new() };
                        bits.extend(padded(cpu(true, self.mark > 0).to_bits(), 24));
                        bits.extend(suvu(true));
                        let v = self.s.trn2u_bits(&bits, false);
                        self.values(v);
                        self.st = St::Ack;
                    } else {
                        let v = self.s.trn2u_bits(&suvu(false), false);
                        self.values(v);
                    }
                }
                St::Ack => {
                    if matches!(m.source.out, Out::Ed | Out::B1d | Out::Data) || (self.e2u_first && m.source.acknowledged >= 1) {
                        let v = self.s.trn2u_bits(&[false; 24], false);
                        self.values(v);
                        let (_, d) = m.design.clone().expect("a design before Ed");
                        self.tx = Some(Transmitter::new(UpParams::from_cpd(&d.cpd).unwrap()));
                        self.st = St::B1u;
                    } else {
                        let v = self.s.trn2u_bits(&suvu(true), false);
                        self.values(v);
                    }
                }
                St::B1u => {
                    let tx = self.tx.as_mut().unwrap();
                    let k = tx_k(tx);
                    for _ in 0..B1U_FRAMES {
                        let s = tx.frame(&vec![true; k]).unwrap().samples;
                        self.queue.extend(s.iter().map(|&v| Sym::V(v)));
                    }
                    self.st = St::Data;
                }
                St::Data if self.renegs_left > 0
                    && (matches!(m.source.out, Out::RdBar | Out::Trn2d) || (self.starts && self.payload.len() > 12_000)) =>
                {
                    // 9.8.2.2.2 answering R-bar-d, or 9.8.2.1.1 of its own
                    // accord: Ru and R-bar-u from a frame boundary, then
                    // TRN2u and Phase 4's exchange again.
                    self.renegs_left -= 1;
                    self.mark = self.payload.len();
                    self.values(ru(384, false));
                    self.values(ru(24, true));
                    self.st = St::Trn2u;
                }
                St::Data => {
                    let tx = self.tx.as_mut().unwrap();
                    let k = tx_k(tx);
                    let bits: Vec<bool> = (0..k)
                        .map(|_| {
                            self.rng ^= self.rng << 13;
                            self.rng ^= self.rng >> 7;
                            self.rng ^= self.rng << 17;
                            self.rng & 1 == 1
                        })
                        .collect();
                    self.payload.extend(&bits);
                    let s = tx.frame(&bits).unwrap().samples;
                    self.queue.extend(s.iter().map(|&v| Sym::V(v)));
                }
            }
        }

        /// The codec's sample at `n`, normalised as the Machine takes it.
        fn sample(&mut self, n: usize, m: &Machine) -> f64 {
            while self.at <= n as f64 {
                match self.queue.pop_front() {
                    Some(Sym::V(v)) => {
                        self.sent.push((self.at, v));
                        self.at += 1.0;
                    }
                    Some(Sym::Wait(w)) => self.at += w,
                    None => self.refill(m),
                }
            }
            let y: f64 = self
                .sent
                .iter()
                .rev()
                .take_while(|(t, _)| *t >= n as f64 - 31.0)
                .map(|&(t, v)| v * band(n as f64 - t))
                .sum::<f64>()
                * 2800.0;
            if self.sent.len() > 4096 {
                self.sent.drain(..2048);
            }
            let (u, negative) = ucode::nearest(self.law, y.round() as i32);
            f64::from(ucode::amplitude(self.law, u, negative)) / 32768.0
        }
    }

    fn tx_k(tx: &Transmitter) -> usize {
        // K from a probe frame length: Transmitter takes exactly K bits.
        (1..200).find(|&k| tx.clone().frame(&vec![false; k]).is_some()).unwrap()
    }

    fn settings(law: Law) -> Settings {
        Settings {
            law,
            uinfo: 73,
            upstream: Band::new(SymbolRate::S3200, false),
            far_md: 0,
            round_trip: 0.0,
            jd: Jd { rates: Jd::ALL_RATES, sixteen_in_training: false, sixteen_in_renegotiation: false, lookahead: 1 },
            wide: false,
            habits: Habits::PROMPT,
        }
    }

    fn request() -> PcmUpstreamRequest {
        PcmUpstreamRequest { filter_sections: 0, total_coefficient_limit: 192, section_coefficient_limit: 128, md_units: 0, uinfo: 73 }
    }

    fn run(law: Law, dil: Descriptor) {
        run_with(law, dil, None, 0, false, false);
    }

    /// `reneg`: a rate renegotiation once data is flowing, begun by the
    /// analogue modem (true) or by this end (false).
    fn run_with(law: Law, dil: Descriptor, reneg: Option<bool>, slow_su: u64, slip: bool, late_cpu: bool) {
        run_modem(law, dil, reneg, slow_su, slip, late_cpu, false, None);
    }

    #[allow(clippy::too_many_arguments)]
    fn run_modem(law: Law, dil: Descriptor, reneg: Option<bool>, slow_su: u64, slip: bool, late_cpu: bool, e2u_first: bool, ed_aim: Option<f64>) {
        let mut m = Machine::new(&settings(law), request());
        m.reneg_ed_aim = ed_aim;
        let mut a = Analogue {
            law,
            at: 0.0,
            queue: VecDeque::new(),
            st: St::Start,
            s: Sender::new(),
            sent: Vec::new(),
            rng: 99,
            since: 0,
            first: true,
            tx: None,
            payload: Vec::new(),
            dil,
            renegs_left: u32::from(reneg.is_some()),
            starts: reneg == Some(true),
            mark: 0,
            slow_su,
            jd_seen: 0,
            late_cpu,
            e2u_first,
        };
        let mut ed_before_e2u = Vec::new();
        let mut was_ed = false;
        let mut asked = false;
        let mut received = Vec::new();
        let mut trace = Vec::new();
        let mut last = St::Start;
        // Downstream: a known pattern kept in the transmitter in data mode,
        // and every sample this end puts on the line.
        let down_bit = |i: usize| {
            // splitmix64: no shift of it is itself or its inverse
            let mut z = (i as u64).wrapping_add(0x9E37_79B9_7F4A_7C15);
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            (z ^ (z >> 31)) & 1 == 1
        };
        let (mut down_sent, mut down_mark, mut datas, mut was_data) = (0usize, None, 0, false);
        let mut line = Vec::new();
        let mut slip_mark = None;
        for n in 0..24 * 8000 {
            let x = a.sample(n, &m);
            if reneg == Some(false) && !asked && a.payload.len() > 12_000 && m.stage == Stage::Data {
                asked = m.renegotiate("asked for by the test");
            }
            let data = m.stage == Stage::Data;
            if data && !was_data {
                datas += 1;
                if datas == 2 {
                    down_mark = Some(down_sent);
                }
            }
            was_data = data;
            if data && m.pending_bits() < 600 {
                let more: Vec<bool> = (down_sent..down_sent + 600).map(down_bit).collect();
                m.send_bits(&more);
                down_sent += 600;
            }
            if slip && slip_mark.is_none() && data && down_sent >= 9000 {
                assert!(m.slip_downstream(), "no slip in data mode");
                slip_mark = Some(down_sent);
            }
            line.push(m.step(x));
            // The decision, not the start: Ed waits for the SUVd' going out.
            let ed = m.source.pending == Some(Out::Ed) || m.source.out == Out::Ed;
            if ed && !was_ed && m.b1u_at.is_none() {
                ed_before_e2u.push(n);
            }
            was_ed = ed;
            trace.extend(m.take_trace());
            received.extend(m.take_bits());
            if a.st != last {
                trace.push(format!("{n}: analogue {:?}", a.st));
                last = a.st;
            }
            if m.take_retrain() || matches!(m.status(), Status::Failed(_)) {
                panic!("{:?}\n{}", m.status(), trace.join("\n"));
            }
            let down_done = if slip { slip_mark.is_some_and(|mk| down_sent > mk + 4000) } else { reneg.is_none() || down_mark.is_some_and(|mk| down_sent > mk + 8000) };
            if down_done && a.renegs_left == 0 && a.payload.len() > a.mark + 20_000 && received.len() > 10_000 && m.stage == Stage::Data {
                break;
            }
        }
        let joined = trace.join("\n");
        assert!(matches!(m.status(), Status::Connected { .. }), "{:?}\n{joined}", m.status());
        if e2u_first && ed_aim.is_none() {
            assert!(ed_before_e2u.is_empty(), "Ed went out before E2u at samples {ed_before_e2u:?}\n{joined}");
        }
        if ed_aim.is_some() {
            assert!(joined.contains("aimed Ed"), "no aimed Ed\n{joined}");
        }
        // 2026-11-16: TRN2d ran 0.83 s in the CX93001's own renegotiations,
        // after which it was deaf, and 1.67 s in ours, after which it heard.
        if reneg.is_some() {
            let at = |l: &str| l.split("pcm-up ").nth(1).and_then(|r| r.split('s').next()).and_then(|t| t.parse::<f64>().ok());
            let lines: Vec<&str> = trace.iter().map(String::as_str).collect();
            let start = lines.iter().position(|l| l.contains("rate renegotiation begun")).expect("a renegotiation");
            let from = lines[start..].iter().find(|l| l.ends_with("going out: TRN2d")).and_then(|l| at(l)).expect("TRN2d in the renegotiation");
            let to = lines[start..].iter().find(|l| l.ends_with("going out: MP")).and_then(|l| at(l)).expect("MP in the renegotiation");
            assert!(to - from >= RENEG_TRN2D_DEFAULT - 0.01 && to - from <= 2.001, "TRN2d in the renegotiation ran {:.3} s", to - from);
        }
        // The payload, somewhere in what was read, whole.
        let want = &a.payload[..4000];
        let found = (0..received.len().saturating_sub(want.len())).find(|&i| &received[i..i + want.len()] == want);
        eprintln!("{joined}");
        assert!(found.is_some(), "payload not found in {} bits", received.len());
        if reneg.is_some() {
            // What the analogue modem sent after the renegotiation, read too:
            // the first data mode alone satisfies the check above.
            let after = &a.payload[a.mark + 3000..a.mark + 7000];
            let read_after = (0..received.len().saturating_sub(after.len())).any(|i| &received[i..i + after.len()] == after);
            assert!(read_after, "nothing sent after the renegotiation was read\n{joined}");
        }
        if slip {
            // One downstream symbol left out: everything after it reaches the
            // analogue modem one symbol early, so its data frames begin one
            // sample sooner on the line.
            let mark = slip_mark.expect("slipped");
            let mapping = Mapping::from_cp(&to_cp(m.cpu.as_ref().expect("CPu"))).expect("mapping");
            let phase_of = |want: &[bool]| -> Vec<usize> {
                (0..INTERVALS)
                    .filter(|&p| {
                        let mut decoder = Decoder::new(mapping.clone());
                        let mut descrambler = Scrambler::new(Mode::Call);
                        let bits: Vec<bool> = line[p..]
                            .chunks_exact(INTERVALS)
                            .flat_map(|f| {
                                let mut ucodes = [0u8; INTERVALS];
                                let mut positive = [false; INTERVALS];
                                for (i, v) in f.iter().enumerate() {
                                    let (u, negative) = ucode::nearest(law, (v * 32768.0).round() as i32);
                                    ucodes[i] = u;
                                    positive[i] = !negative;
                                }
                                decoder.frame(Frame { ucodes, positive }).into_iter().map(|b| descrambler.descramble(b)).collect::<Vec<_>>()
                            })
                            .collect();
                        (0..bits.len().saturating_sub(want.len())).any(|i| bits[i..i + want.len()] == want[..])
                    })
                    .collect()
            };
            let before = phase_of(&(mark - 3000..mark - 2000).map(down_bit).collect::<Vec<_>>());
            let after = phase_of(&(mark + 2000..mark + 3000).map(down_bit).collect::<Vec<_>>());
            assert_eq!(before.len(), 1, "{before:?}");
            assert_eq!(after, vec![(before[0] + INTERVALS - 1) % INTERVALS], "before {before:?}");
            return;
        }
        if reneg.is_some() {
            // The renegotiation happened, and data came through after it.
            assert!(joined.contains("Ru-to-R-bar-u"), "{joined}");
            assert!(a.mark > 0, "{joined}");
            let want = &a.payload[a.mark + 500..a.mark + 3500];
            let found = (0..received.len().saturating_sub(want.len())).find(|&i| &received[i..i + want.len()] == want);
            assert!(found.is_some(), "payload after the renegotiation not found\n{joined}");
            // And the analogue modem can still read this end: the pattern
            // sent downstream after the renegotiation, decoded from the line
            // at the data frames' phase as 5.4's decoder reads it.
            let mark = down_mark.expect("data mode twice");
            let mapping_of = |c: &UpCp| Mapping::from_cp(&to_cp(c)).expect("mapping");
            let after: Vec<bool> = (mark + 1500..mark + 2500).map(down_bit).collect();
            let decoded = |mapping: &Mapping, phase: usize| -> Vec<bool> {
                let mut decoder = Decoder::new(mapping.clone());
                let mut descrambler = Scrambler::new(Mode::Call);
                line[phase..]
                    .chunks_exact(INTERVALS)
                    .flat_map(|f| {
                        let mut ucodes = [0u8; INTERVALS];
                        let mut positive = [false; INTERVALS];
                        for (i, v) in f.iter().enumerate() {
                            let (u, negative) = ucode::nearest(law, (v * 32768.0).round() as i32);
                            ucodes[i] = u;
                            positive[i] = !negative;
                        }
                        decoder.frame(Frame { ucodes, positive }).into_iter().map(|b| descrambler.descramble(b)).collect::<Vec<_>>()
                    })
                    .collect()
            };
            let phases = |mapping: &Mapping, want: &[bool]| -> Vec<usize> {
                (0..INTERVALS)
                    .filter(|&p| {
                        let bits = decoded(mapping, p);
                        (0..bits.len().saturating_sub(want.len())).any(|i| bits[i..i + want.len()] == want[..])
                    })
                    .collect()
            };
            let now = phases(&mapping_of(m.cpu.as_ref().expect("CPu")), &after);
            assert!(!now.is_empty(), "downstream data after the renegotiation does not decode at any frame phase\n{joined}");
            // At the same phase as before it. The analogue modem found the
            // data frames' boundaries once, in start-up, and a renegotiation
            // does not find them again: data that comes back on a different
            // grid is read wrongly from the first frame.
            let before: Vec<bool> = (200..1200).map(down_bit).collect();
            let then = phases(&mapping_of(m.cpu_before.as_ref().expect("CPu before")), &before);
            assert!(!then.is_empty(), "downstream data before the renegotiation does not decode\n{joined}");
            assert_eq!(then, now, "the downstream frame grid moved across the renegotiation");
            return;
        }
        // Phase 4 measured TRN2u and the CPd came from that (8.7.6, 9.6.1.1.1).
        assert!(joined.contains("design on TRN2u: noise"), "{joined}");
        assert!(!joined.contains("stands"), "{joined}");
        // One CPd, and after the design it carries.
        let cpd = joined.find("CPd going out").expect("CPd");
        assert!(joined.find("design on TRN2u: noise").is_some_and(|d| d < cpd), "{joined}");
    }

    /// A CPu as the "CPu bits (N): hex" log line gives it.
    fn cpu_from_hex(hex: &str, len: usize) -> UpCp {
        let mut bits: Vec<bool> = hex.chars().flat_map(|c| { let v = c.to_digit(16).unwrap(); (0..4).rev().map(move |i| v >> i & 1 == 1) }).collect();
        let tail = len % 4;
        if tail != 0 { let n = bits.len(); let last: Vec<bool> = bits[n - tail..].to_vec(); bits.truncate(n - 4); bits.extend(last); }
        let Some(UpInfo::Cp(c)) = UpInfo::from_bits(&bits) else { panic!("not a CPu") };
        c
    }

    #[test]
    fn a_cpu_whose_second_set_is_empty_uses_the_first_there() {
        // og5 call 1, 2026-11-13: a Conexant's CPu in a rate renegotiation,
        // CRC good, drn 10, intervals 0 and 1 on set 1, set 1 empty. 40000
        // bit/s needs K = 25: 18 points in all six intervals (18^6 >= 2^25)
        // and nothing less, so set 1 can only mean set 0.
        let c = cpu_from_hex("ffff9281000040004010000001100000000000080002008084212492aaa00000000000000000000000000000000000000000005d7d0", 426);
        assert_eq!(c.constellations[1], 0);
        let mapping = Mapping::from_cp(&to_cp(&c)).expect("refused");
        assert_eq!(mapping.moduli(), [18; INTERVALS]);
        assert_eq!(mapping.rate(), 40_000);
    }

    /// Read our downstream back off a call recording (MixMonitor's t leg,
    /// 16-bit linear) with the mapping a logged CPu asked for, and count
    /// HDLC frames whose FCS checks, per frame phase, per second.
    /// REC=path CPU=hex CPU_LEN=bits FROM=s TO=s cargo test ... -- --ignored
    #[test]
    #[ignore]
    fn read_downstream_off_a_recording() {
        let var = |k: &str| std::env::var(k).unwrap_or_else(|_| panic!("{k}"));
        let raw = std::fs::read(var("REC")).unwrap();
        let x: Vec<i32> = raw.chunks_exact(2).map(|b| i32::from(i16::from_le_bytes([b[0], b[1]]))).collect();
        let hex = var("CPU");
        let len: usize = var("CPU_LEN").parse().unwrap();
        let mut bits: Vec<bool> = hex.chars().flat_map(|c| { let v = c.to_digit(16).unwrap(); (0..4).rev().map(move |i| v >> i & 1 == 1) }).collect();
        // The last nibble was folded from fewer than four bits.
        let tail = len % 4;
        if tail != 0 { let n = bits.len(); let last: Vec<bool> = bits[n - tail..].to_vec(); bits.truncate(n - 4); bits.extend(last); }
        assert_eq!(bits.len(), len);
        let Some(UpInfo::Cp(c)) = UpInfo::from_bits(&bits) else { panic!("not a CPu") };
        eprintln!("CPu: drn {} Sr {:?} ld {} shaping {:?} intervals {:?} trn1d {} a_law {} constellations {}", c.drn, c.redundancy, c.lookahead, c.shaping, c.intervals, c.trn1d_ratio, c.a_law, c.constellations.len());
        for (i, m) in c.constellations.iter().enumerate() { eprintln!("  set {i}: {:032x} ({} points)", m, m.count_ones()); }
        if let Some(codec) = &c.codec { for (i, m) in codec.iter().enumerate() { eprintln!("  codec {i}: {:032x} ({} points)", m, m.count_ones()); } }
        let mapping = Mapping::from_cp(&to_cp(&c)).expect("mapping");
        let from = (var("FROM").parse::<f64>().unwrap() * 8000.0) as usize;
        let to = ((var("TO").parse::<f64>().unwrap() * 8000.0) as usize).min(x.len());
        let law = if c.a_law { Law::A } else { Law::Mu };
        for phase in 0..INTERVALS {
            let mut decoder = Decoder::new(mapping.clone());
            let mut descrambler = Scrambler::new(Mode::Call);
            let mut out: Vec<(usize, bool)> = Vec::new();
            for (k, f) in x[from + phase..to].chunks_exact(INTERVALS).enumerate() {
                let mut ucodes = [0u8; INTERVALS];
                let mut positive = [false; INTERVALS];
                for (i, v) in f.iter().enumerate() {
                    let (u, negative) = ucode::nearest(law, *v);
                    ucodes[i] = u;
                    positive[i] = !negative;
                }
                for b in decoder.frame(Frame { ucodes, positive }) {
                    out.push((from + phase + k * INTERVALS, descrambler.descramble(b)));
                }
            }
            // HDLC: flags, destuffing, FCS-16 (good residual 0xf0b8).
            let mut good: std::collections::BTreeMap<usize, (u32, u32)> = Default::default();
            let (mut ones, mut frame, mut inside) = (0u32, Vec::<bool>::new(), false);
            for &(at, b) in &out {
                if b { ones += 1; if ones > 6 { inside = false; frame.clear(); } frame.push(b); continue; }
                if ones == 6 {
                    // A flag: close what was open.
                    frame.truncate(frame.len().saturating_sub(7));
                    if inside && frame.len() >= 32 && frame.len() % 8 == 0 {
                        let mut fcs: u16 = 0xffff;
                        for &bit in &frame { let x = (fcs ^ u16::from(bit)) & 1; fcs >>= 1; if x == 1 { fcs ^= 0x8408; } }
                        let e = good.entry(at / 8000).or_default();
                        if fcs == 0xf0b8 { e.0 += 1 } else { e.1 += 1 }
                    }
                    frame.clear(); inside = true; ones = 0; continue;
                }
                if ones == 5 { ones = 0; continue; } // stuffed zero
                ones = 0;
                frame.push(b);
            }
            let total: u32 = good.values().map(|v| v.0).sum();
            if total > 0 {
                let line: Vec<String> = good.iter().map(|(s, (g, b))| format!("{s}:{g}/{b}")).collect();
                eprintln!("phase {phase}: {total} good frames; per second good/bad {}", line.join(" "));
            } else {
                eprintln!("phase {phase}: none");
            }
        }
    }

    #[test]
    fn a_renegotiation_waits_for_the_new_cpu_before_ed() {
        run_with(Law::Mu, Descriptor::none(), Some(true), 0, false, true);
    }

    /// The renegotiation of 2026-11-13, every one on the outside route: the
    /// Conexant's SUVu' came before its E2u, this end sent Ed at once, and
    /// the modem never read our data after. In start-up the same modem
    /// sends E2u first and is heard every time; a renegotiation now waits
    /// for E2u the same way.
    #[test]
    fn a_renegotiation_sends_ed_only_after_e2u_from_a_modem_that_sends_it_first() {
        run_modem(Law::Mu, Descriptor::none(), Some(true), 0, false, true, true, None);
    }

    #[test]
    fn a_renegotiation_this_end_begins_sends_ed_only_after_e2u() {
        run_modem(Law::Mu, Descriptor::none(), Some(false), 0, false, false, true, None);
    }

    #[test]
    fn a_downstream_slip_moves_the_data_frames_one_symbol_sooner() {
        run_with(Law::Mu, Descriptor::none(), None, 0, true, false);
    }

    #[test]
    fn a_scripted_analogue_modem_asking_no_dil_reaches_pcm_upstream_data() {
        run(Law::Mu, Descriptor::none());
    }

    /// The DIL descriptor a real Conexant V.92 sent (sequences.rs's
    /// recording): 147 segments, 1736 bits, so Ja is longer than that.
    fn conexant() -> Descriptor {
        let bits = |s: &str| s.bytes().map(|b| b == b'1').collect::<Vec<bool>>();
        let signs = bits("000111011000101001011111010101000010110111100111001010110011000001101101011101000110010001000000100100110100111101110000111111");
        let training = bits("000000000000000000000000000000101010010101000000000000101010010101000000111111111111111111111111111111111111111111111111111111");
        let mut ucodes = Vec::new();
        for u in 0..118u8 {
            ucodes.push(u);
            if ucodes.len() % 5 == 4 {
                ucodes.push(78);
            }
        }
        ucodes.truncate(147);
        Descriptor { signs, training, h: [20, 20, 20, 20, 20, 20, 11, 11], refs: [78; 8], ucodes }
    }

    #[test]
    fn a_real_conexant_s_long_ja_is_read() {
        run(Law::Mu, conexant());
    }

    #[test]
    fn a_rate_renegotiation_this_end_begins_returns_to_data() {
        run_with(Law::Mu, Descriptor::none(), Some(false), 0, false, false);
    }

    /// 2026-11-16: the CX93001 sends E2u about 0.76 s after its first CPu
    /// and was deaf after 14 of its 17 renegotiations with Ed going out on
    /// E2u heard, a round trip (about 1 s) after its E2u. An aimed Ed goes
    /// out before E2u is heard and the renegotiation still reaches data
    /// both ways, at the CPu's mapping.
    #[test]
    fn an_aimed_ed_in_a_renegotiation_the_analogue_modem_begins_returns_to_data() {
        run_modem(Law::Mu, Descriptor::none(), Some(true), 0, false, false, true, Some(-0.3));
    }

    #[test]
    fn a_rate_renegotiation_the_analogue_modem_begins_returns_to_data() {
        run_with(Law::Mu, Descriptor::none(), Some(true), 0, false, false);
    }

    /// Live, 2026-11-12: the Conexant's Su came about 5.5 s after TRN1d began
    /// where it usually comes at 4.9 s, a tenth of a second after this end
    /// had given up at 5100 ms and a round trip; the call fell back to V.34
    /// upstream. Here Su comes 5.8 s into Jd, past the 5100 ms.
    #[test]
    fn an_analogue_modem_slow_with_its_su_is_waited_for() {
        run_with(Law::Mu, Descriptor::none(), None, 46_400, false, false);
    }

    #[test]
    fn with_a_dil_and_a_law_the_same() {
        run(Law::A, crate::v90::dil::design(Law::A, 73));
    }
}

#[cfg(test)]
mod explain_cpu {
    use super::*;
    #[test]
    #[ignore]
    fn explain() {
        let hex = std::env::var("CPU").unwrap();
        let mut bits: Vec<bool> = hex.chars().flat_map(|c| { let v = c.to_digit(16).unwrap(); (0..4).rev().map(move |i| v >> i & 1 == 1) }).collect();
        let len: usize = std::env::var("CPU_LEN").unwrap().parse().unwrap();
        let tail = len % 4;
        if tail != 0 { let n = bits.len(); let last: Vec<bool> = bits[n - tail..].to_vec(); bits.truncate(n - 4); bits.extend(last); }
        let Some(UpInfo::Cp(c)) = UpInfo::from_bits(&bits) else { panic!("not a CPu") };
        let cp = to_cp(&c);
        eprintln!("{}", cp.describe());
        for i in 0..6 { eprintln!("interval {i}: set {} -> {} points", cp.intervals[i], cp.points(i).len()); }
        for (n, m) in cp.constellations.iter().enumerate() { eprintln!("set {n}: {} points {:?}", m.count_ones(), (0..128u32).filter(|u| *m >> u & 1 == 1).collect::<Vec<_>>()); }
        eprintln!("interval0 points {:?}", cp.points(0));
        if let Some(codec) = &cp.codec { for (n, m) in codec.iter().enumerate() { eprintln!("codec set {n}: {} {:?}", m.count_ones(), (0..128u32).filter(|u| *m >> u & 1 == 1).collect::<Vec<_>>()); } }
        eprintln!("frame bits {} S {}", cp.frame_bits(), cp.redundancy.data_bits());
        eprintln!("mapping: {:?}", Mapping::from_cp(&cp).map(|m| m.rate()));
        let k = (cp.frame_bits() - cp.redundancy.data_bits()) as f64;
        let lg: f64 = (0..6).map(|i| (cp.points(i).len() as f64).log2()).sum();
        let lg1: f64 = (0..6).map(|i| (cp.points(i).len() as f64 + 1.0).log2()).sum();
        eprintln!("MARGIN drn {} K {k} margin {:.3} margin_plus1 {:.3}", cp.drn, lg - k, lg1 - k);
    }
}
