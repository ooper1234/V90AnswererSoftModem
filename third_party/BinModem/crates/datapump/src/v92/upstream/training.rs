//! The upstream half of V.92 Phase 3 and 4 when PCM upstream is selected:
//! the analogue modem's training signals (8.5, 8.7), sent around the
//! precoder and prefilter at 8000 symbols a second, in units of LU.
//!
//! Two modulations carry bits. Ja, CPt and E1u use TRN1u's: two points, the
//! bits scrambled and then differentially encoded, 0 positive and 1
//! negative. SUVu, CPu and E2u use TRN2u's: 4 or 8 points (Tables 28, 29),
//! the scrambler's bits taken least significant first into a symbol whose
//! most significant bit is the sign, and only the sign differentially
//! encoded.

use crate::v32::{Mode, Scrambler};
use crate::v92::phase3::{su as su_pattern, RU};

/// Ru (8.5.5), or R̄u, `len` symbols.
pub fn ru(len: usize, bar: bool) -> Vec<f64> {
    let s = if bar { -1.0 } else { 1.0 };
    (0..len).map(|n| s * RU[n % 6]).collect()
}

/// Su (8.5.6), or S̄u, `len` symbols.
pub fn su(len: usize, bar: bool) -> Vec<f64> {
    let s = if bar { -1.0 } else { 1.0 };
    let p = su_pattern();
    (0..len).map(|n| s * p[n % 6]).collect()
}

/// Bits per TRN2u symbol.
pub fn trn2u_bits_per_symbol(eight: bool) -> usize {
    if eight { 3 } else { 2 }
}

/// A TRN2u symbol's level from its magnitude bits and sign.
pub fn trn2u_level(magnitude: u8, negative: bool, eight: bool) -> f64 {
    let scale = if eight { 21f64 } else { 5f64 }.sqrt();
    let v = f64::from(2 * magnitude + 1) / scale;
    if negative { -v } else { v }
}

/// The analogue modem's training transmitter. It keeps the scrambler and
/// the differential encoder's memory -- the last sign sent, true for
/// negative -- from one signal to the next, as 8.5.1, 8.5.4 and 8.7.6 ask.
#[derive(Debug, Clone)]
pub struct Sender {
    scrambler: Scrambler,
    negative: bool,
}

impl Default for Sender {
    fn default() -> Self {
        Self::new()
    }
}

impl Sender {
    pub fn new() -> Self {
        Self { scrambler: Scrambler::new(Mode::Answer), negative: false }
    }

    /// TRN1u (8.5.7): the scrambler from zero, fed ones; 0 is +LU, 1 is -LU.
    pub fn trn1u(&mut self, len: usize) -> Vec<f64> {
        self.scrambler = Scrambler::new(Mode::Answer);
        (0..len)
            .map(|_| {
                self.negative = self.scrambler.scramble(true);
                if self.negative { -1.0 } else { 1.0 }
            })
            .collect()
    }

    /// Bits in TRN1u's modulation (Ja, CPt, E1u): scrambled, then added
    /// modulo 2 to the bit sent before.
    pub fn two_point(&mut self, bits: &[bool]) -> Vec<f64> {
        bits.iter()
            .map(|&b| {
                self.negative ^= self.scrambler.scramble(b);
                if self.negative { -1.0 } else { 1.0 }
            })
            .collect()
    }

    /// TRN2u (8.7.6): scrambled ones, the scrambler reset first.
    pub fn trn2u(&mut self, len: usize, eight: bool) -> Vec<f64> {
        self.scrambler = Scrambler::new(Mode::Answer);
        let ones = vec![true; len * trn2u_bits_per_symbol(eight)];
        self.trn2u_bits(&ones, eight)
    }

    /// Bits in TRN2u's modulation (SUVu, CPu, E2u). A last symbol short of
    /// bits is filled with zeros.
    pub fn trn2u_bits(&mut self, bits: &[bool], eight: bool) -> Vec<f64> {
        let k = trn2u_bits_per_symbol(eight);
        bits.chunks(k)
            .map(|chunk| {
                let mut word = 0u8;
                for i in 0..k {
                    if self.scrambler.scramble(chunk.get(i).copied().unwrap_or(false)) {
                        word |= 1 << i;
                    }
                }
                self.negative ^= word >> (k - 1) & 1 == 1;
                trn2u_level(word & ((1 << (k - 1)) - 1), self.negative, eight)
            })
            .collect()
    }
}

/// What the [`Listener`] has made out.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Heard {
    /// The Ru-to-R̄u transition (the last one, after MD).
    RuReversal,
    /// TRN1u has trained the receiver; its decisions' signal to error ratio.
    Trained { snr_db: f64 },
}

/// TRN1u symbols the channel is estimated over: the least 8.5.7 allows.
const TRAIN: usize = 2040;
/// Taps of the estimated channel; it starts `LEAD` symbols before TRN1u is
/// expected, so the estimate's own delay takes up the reversal's
/// uncertainty and the channel's delay.
const TAPS: usize = 80;
const LEAD: usize = 32;
/// TRN1u's first symbols the second one is found by, and how strong the
/// match must be: 1 is a line of one tap and no noise, chance is 1/sqrt(FIND).
const FIND: usize = 512;
const FOUND: f64 = 0.3;
/// Where in the estimate the main tap is put when TRN1u was found by it:
/// room for the response before it and after.
const MAIN: usize = 36;
/// Receive equaliser: forward and decision-feedback lengths.
const EQUALISER: usize = 64;
const FEEDBACK: usize = 32;
/// Mean square below which nothing is taken for Ru (codec units).
const FLOOR: f64 = 1000.0;
/// Symbols of Ru the Listener wants before it looks for the reversal.
const RU_HELD: usize = 96;

#[derive(Debug, Clone)]
enum Phase {
    /// Looking for Ru; `held` symbols of it so far; `skip` symbols of MD
    /// still to let go by.
    Seeking { held: usize, skip: usize },
    /// Ru heard; waiting for the reversal.
    Reversal,
    /// Collecting TRN1u from `base`.
    Collecting { base: usize },
    /// Looking for the second TRN1u's first symbols from sample `from`: the
    /// best match so far as (strength, sample), if one has crossed.
    Finding { from: usize, best: Option<(f64, usize)> },
    /// TRN1u is in; the equaliser is being designed a few delays a sample
    /// (the [`Training`] beside the phase).
    Training,
    Trained,
}

/// The receive equaliser for TRN1u, designed across samples rather than in
/// one: four regularisations of 64 delays, each delay one solve, was 60-90
/// ms of a single sample on the server and a gap the line heard. One delay
/// a sample takes 4 x 64 samples (32 ms) and gives what the one-shot design
/// gave.
#[derive(Debug, Clone)]
struct Training {
    base: usize,
    /// Samples in when TRN1u was: each regularisation is judged on these
    /// alone, as the one-shot design judged them, not on what came since.
    end: usize,
    channel: super::design::Channel,
    signs: Vec<f64>,
    /// Which of RATIOS the job is for.
    ratio: usize,
    job: super::design::EqualiseJob,
    best: Option<(f64, super::design::Equaliser)>,
}

/// Regularisations tried, the one that decides TRN1u best kept.
const RATIOS: [f64; 4] = [1e-3, 1e-4, 1e-5, 1e-6];
/// Equaliser delays tried per sample while training.
const DELAYS_PER_SAMPLE: usize = 1;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Modulation {
    TwoPoint,
    Trn2u { eight: bool },
}

/// A transition in Su (8.5.6, 9.5.1.1.6-9), at the first symbol of the
/// signal it turns into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SuTurn {
    ToBar(usize),
    ToSu(usize),
}

/// The digital modem's receiver for the upstream training signals: it finds
/// the Ru-to-R̄u transition, trains a channel estimate and an equaliser on
/// TRN1u, and then reads bits -- Ja, CPt and E1u in TRN1u's modulation,
/// SUVu, CPu and E2u in TRN2u's after [`Self::switch_trn2u`]. Symbol 0 is
/// TRN1u's first.
#[derive(Debug, Clone)]
pub struct Listener {
    law: crate::v90::ucode::Law,
    y: Vec<f64>,
    phase: Phase,
    /// Symbols of MD to skip after the first reversal; none when INFO1a gave
    /// MD no length.
    md: usize,
    /// TRN1u's estimate of the analogue modem's path to the codec.
    channel: Option<super::design::Channel>,
    w: Vec<f64>,
    /// Decision feedback: minus the combined response after the main tap.
    p1: Vec<f64>,
    delay: usize,
    /// Every symbol decided so far, in LU, for the feedback; and what the
    /// equaliser gave for each before the decision.
    decided: Vec<f64>,
    softs: Vec<f64>,
    base: usize,
    /// Next symbol to decide; and from where on the modulation is which.
    next: usize,
    segments: Vec<(usize, Modulation)>,
    /// TRN1u's end, once decisions stop following its known signs.
    trn1u_end: Option<usize>,
    trn1u: Scrambler,
    mismatches: std::collections::VecDeque<bool>,
    descrambler: Scrambler,
    /// The last decided sign, true for negative.
    negative: bool,
    bits: Vec<bool>,
    /// The symbol each untaken bit came from.
    bit_symbols: Vec<usize>,
    /// Su's watch: consecutive symbols that kept their sign from six
    /// before, whether S̄u is what is being sent, and turns not yet taken.
    su_same: usize,
    su_bar: bool,
    su_turns: Vec<SuTurn>,
    su_watching: bool,
    find_signs: Vec<f64>,
    su_ref: (f64, f64),
    su_flip: Option<(usize, usize)>,
    su_quiet: usize,
    /// The strongest line-up of TRN1u's first signs since the search for it
    /// began, and where: for saying why a search found nothing.
    find_seen: (f64, usize),
    /// The equaliser design in progress (Phase::Training).
    training: Option<Box<Training>>,
    /// Tests: train in one go, as before the design was spread out.
    #[cfg(test)]
    at_once: bool,
}

impl Listener {
    /// `md_symbols`: MD's length from INFO1a, in symbols; 0 when there is none.
    pub fn new(law: crate::v90::ucode::Law, md_symbols: usize) -> Self {
        Self {
            law,
            y: Vec::new(),
            phase: Phase::Seeking { held: 0, skip: 0 },
            md: md_symbols,
            channel: None,
            w: Vec::new(),
            p1: Vec::new(),
            delay: 0,
            decided: Vec::new(),
            softs: Vec::new(),
            base: 0,
            next: 0,
            segments: vec![(0, Modulation::TwoPoint)],
            trn1u_end: None,
            trn1u: Scrambler::new(Mode::Answer),
            mismatches: std::collections::VecDeque::new(),
            descrambler: Scrambler::new(Mode::Answer),
            negative: false,
            bits: Vec::new(),
            bit_symbols: Vec::new(),
            su_same: 0,
            su_bar: false,
            su_turns: Vec::new(),
            su_watching: false,
            find_signs: super::design::trn1u_signs(FIND),
            su_ref: (0.0, 0.0),
            su_flip: None,
            su_quiet: 0,
            find_seen: (0.0, 0),
            training: None,
            #[cfg(test)]
            at_once: false,
        }
    }

    pub fn channel(&self) -> Option<&super::design::Channel> {
        self.channel.as_ref()
    }

    /// One codec octet as received.
    pub fn push(&mut self, octet: u8) -> Option<Heard> {
        let (u, negative) = crate::v90::ucode::from_octet(self.law, octet);
        let v = f64::from(crate::v90::ucode::linear(self.law, u));
        self.push_linear(if negative { -v } else { v })
    }

    /// One codec sample in linear units.
    pub fn push_linear(&mut self, sample: f64) -> Option<Heard> {
        self.y.push(sample);
        let n = self.y.len() - 1;
        if self.su_watching {
            self.watch_su(n);
        }
        match self.phase.clone() {
            Phase::Seeking { held, skip } => {
                if skip > 0 {
                    self.phase = Phase::Seeking { held: 0, skip: skip - 1 };
                } else if n >= 6 && self.periodic(n) {
                    self.phase = if held + 1 >= RU_HELD { Phase::Reversal } else { Phase::Seeking { held: held + 1, skip: 0 } };
                } else {
                    self.phase = Phase::Seeking { held: 0, skip: 0 };
                }
                None
            }
            Phase::Reversal => {
                let d = self.y[n] - self.y[n - 6];
                let level = self.y[n - RU_HELD.min(n)..n].iter().map(|v| v * v).sum::<f64>() / RU_HELD as f64;
                if d * d < level {
                    return None;
                }
                if self.md > 0 {
                    // The first reversal; MD and the second Ru follow.
                    let md = std::mem::take(&mut self.md);
                    self.phase = Phase::Seeking { held: 0, skip: 24 + md };
                    return None;
                }
                // TRN1u follows R̄u's 24 symbols; the reversal shows a few
                // symbols late, by the channel's delay.
                self.phase = Phase::Collecting { base: (n + 24).saturating_sub(LEAD) };
                Some(Heard::RuReversal)
            }
            Phase::Finding { from, best } => {
                // TRN1u's known first signs against the last FIND samples:
                // where its first symbol's main response has just arrived,
                // they line up, at any gap after S-bar-u (9.5.2.1.9) and
                // whatever else is on the line.
                if n + 1 < from + FIND {
                    return None;
                }
                let at = n + 1 - FIND;
                let (mut c, mut e) = (0.0, 0.0);
                for (i, s) in self.find_signs.iter().enumerate() {
                    let v = self.y[at + i];
                    c += v * s;
                    e += v * v;
                }
                let strength = c.abs() / ((e * FIND as f64).sqrt() + 1e-9);
                if strength > self.find_seen.0 {
                    self.find_seen = (strength, at);
                }
                let best = match best {
                    None if strength > FOUND => Some((strength, at)),
                    Some((s, _)) if strength > s => Some((strength, at)),
                    other => other,
                };
                self.phase = match best {
                    // A little past the crossing, the strongest stands.
                    Some((_, peak)) if at >= peak + 16 => Phase::Collecting { base: peak.saturating_sub(MAIN) },
                    _ => Phase::Finding { from, best },
                };
                None
            }
            Phase::Collecting { base } => {
                if self.y.len() < base + TRAIN {
                    return None;
                }
                #[cfg(test)]
                if self.at_once {
                    return Some(self.train(base));
                }
                self.begin_training(base);
                self.train_step()
            }
            Phase::Training => self.train_step(),
            Phase::Trained => {
                self.decide();
                None
            }
        }
    }

    /// Ru, period 6, around sample `n`: y[n] close to y[n-6] over the last
    /// 12 samples, and loud enough.
    fn periodic(&self, n: usize) -> bool {
        if n < 18 {
            return false;
        }
        let (mut same, mut energy) = (0.0, 0.0);
        for m in n - 11..=n {
            same += (self.y[m] - self.y[m - 6]).powi(2);
            energy += self.y[m].powi(2);
        }
        energy / 12.0 > FLOOR && same < 0.05 * energy
    }

    /// The one-shot training this replaced, for the tests to hold it to.
    #[cfg(test)]
    fn train(&mut self, base: usize) -> Heard {
        let signs = super::design::trn1u_signs(TRAIN);
        let channel = super::design::estimate(&self.y[base..base + TRAIN], &signs, TAPS);
        self.base = base;
        // The regularisation that decides TRN1u best, fed back with the
        // known signs.
        let mut best: Option<(f64, super::design::Equaliser)> = None;
        for ratio in [1e-3, 1e-4, 1e-5, 1e-6] {
            let e = super::design::equalise_with(&channel.h, EQUALISER, FEEDBACK, ratio);
            if !e.residual.is_finite() {
                continue;
            }
            self.w = e.f.clone();
            self.p1 = e.p1.clone();
            self.delay = e.delay;
            let (mut error, mut count) = (0.0, 0usize);
            for (n, s) in signs.iter().enumerate().skip(TAPS) {
                if let Some(z) = self.equalised(n) {
                    error += (z + self.feedback(n, &signs) - s).powi(2);
                    count += 1;
                }
            }
            let mse = error / count.max(1) as f64;
            if best.as_ref().is_none_or(|(b, _)| mse < *b) {
                best = Some((mse, e));
            }
        }
        let (mse, e) = best.unwrap_or((1.0, super::design::equalise_with(&channel.h, EQUALISER, 0, 1e-3)));
        self.w = e.f;
        self.p1 = e.p1;
        self.delay = e.delay;
        self.channel = Some(channel);
        self.phase = Phase::Trained;
        self.decide();
        Heard::Trained { snr_db: -10.0 * mse.max(1e-12).log10() }
    }

    /// TRN1u is in from `base`: the channel estimate at once (a millisecond
    /// or so), the equaliser over the samples that follow.
    fn begin_training(&mut self, base: usize) {
        let signs = super::design::trn1u_signs(TRAIN);
        let channel = super::design::estimate(&self.y[base..base + TRAIN], &signs, TAPS);
        self.base = base;
        let job = super::design::EqualiseJob::new(&channel.h, EQUALISER, FEEDBACK, RATIOS[0]);
        let end = self.y.len();
        self.training = Some(Box::new(Training { base, end, channel, signs, ratio: 0, job, best: None }));
        self.phase = Phase::Training;
    }

    /// A few more of the equaliser's delays; Trained once all are done.
    fn train_step(&mut self) -> Option<Heard> {
        let mut t = self.training.take()?;
        let Some(e) = t.job.step(DELAYS_PER_SAMPLE) else {
            self.training = Some(t);
            return None;
        };
        // One regularisation done: how well it decides TRN1u, fed back with
        // the known signs.
        if e.residual.is_finite() {
            self.w = e.f.clone();
            self.p1 = e.p1.clone();
            self.delay = e.delay;
            let (mut error, mut count) = (0.0, 0usize);
            for (n, s) in t.signs.iter().enumerate().skip(TAPS) {
                if self.base + n + self.delay >= t.end {
                    continue;
                }
                if let Some(z) = self.equalised(n) {
                    error += (z + self.feedback(n, &t.signs) - s).powi(2);
                    count += 1;
                }
            }
            let mse = error / count.max(1) as f64;
            if t.best.as_ref().is_none_or(|(b, _)| mse < *b) {
                t.best = Some((mse, e));
            }
        }
        t.ratio += 1;
        if let Some(&ratio) = RATIOS.get(t.ratio) {
            t.job = super::design::EqualiseJob::new(&t.channel.h, EQUALISER, FEEDBACK, ratio);
            self.training = Some(t);
            return None;
        }
        let Training { base, channel, best, .. } = *t;
        let (mse, e) = best.unwrap_or_else(|| (1.0, super::design::equalise_with(&channel.h, EQUALISER, 0, 1e-3)));
        self.base = base;
        self.w = e.f;
        self.p1 = e.p1;
        self.delay = e.delay;
        self.channel = Some(channel);
        self.phase = Phase::Trained;
        self.decide();
        Some(Heard::Trained { snr_db: -10.0 * mse.max(1e-12).log10() })
    }

    /// The equaliser's output for symbol `n`, if its sample is in.
    fn equalised(&self, n: usize) -> Option<f64> {
        let at = self.base + n + self.delay;
        if at >= self.y.len() {
            return None;
        }
        Some(self.w.iter().enumerate().filter(|(j, _)| *j <= at).map(|(j, w)| w * self.y[at - j]).sum())
    }

    /// What the decisions before symbol `n` add back: the ISI after the
    /// main tap that the feedback cancels.
    fn feedback(&self, n: usize, decided: &[f64]) -> f64 {
        self.p1.iter().enumerate().filter(|(k, _)| *k < n).map(|(k, p)| p * decided.get(n - 1 - k).copied().unwrap_or(0.0)).sum()
    }

    /// Symbol `n`'s soft value, with the decisions before it fed back.
    fn soft(&self, n: usize) -> Option<f64> {
        Some(self.equalised(n)? + self.feedback(n, &self.decided))
    }

    fn record(&mut self, n: usize, value: f64) {
        self.decided.truncate(n);
        self.decided.resize(n, 0.0);
        self.decided.push(value);
    }

    fn modulation(&self, n: usize) -> Modulation {
        self.segments.iter().rev().find(|(from, _)| *from <= n).map(|&(_, m)| m).unwrap_or(Modulation::TwoPoint)
    }

    fn decide(&mut self) {
        if matches!(self.phase, Phase::Training) {
            // No equaliser yet: what is in is decided once it is designed.
            return;
        }
        while let Some(z) = self.soft(self.next) {
            let n = self.next;
            self.softs.truncate(n);
            self.softs.resize(n, 0.0);
            self.softs.push(z);
            match self.modulation(n) {
                Modulation::TwoPoint => {
                    let bit = z < 0.0;
                    self.record(n, if bit { -1.0 } else { 1.0 });
                    match self.trn1u_end {
                        None => {
                            // Still TRN1u while decisions follow its signs.
                            let expected = self.trn1u.scramble(true);
                            self.mismatches.push_back(bit != expected);
                            if self.mismatches.len() > 24 {
                                self.mismatches.pop_front();
                            }
                            let wrong = self.mismatches.iter().filter(|&&m| m).count();
                            if n >= TRAIN / 2 && wrong >= 6 {
                                // The window's first symbol is Ja's first, or near.
                                let end = n + 1 - self.mismatches.len();
                                self.trn1u_end = Some(end);
                                self.descrambler = Scrambler::new(Mode::Answer);
                                // History: TRN1u's line bits, then Ja's
                                // differentially decoded bits from `end`.
                                self.next = end.saturating_sub(24);
                                self.mismatches.clear();
                                self.replay_to(end);
                                continue;
                            }
                            self.negative = bit;
                            self.descrambler.descramble(bit);
                        }
                        Some(_) => {
                            let line = bit ^ self.negative;
                            self.negative = bit;
                            let b = self.descrambler.descramble(line);
                            self.emit(n, b);
                        }
                    }
                }
                Modulation::Trn2u { eight } => {
                    let k = trn2u_bits_per_symbol(eight);
                    let scale = if eight { 21f64 } else { 5f64 }.sqrt();
                    let top = (1u8 << (k - 1)) - 1;
                    let magnitude = (((z.abs() * scale - 1.0) / 2.0).round().max(0.0) as u8).min(top);
                    let negative = z < 0.0;
                    self.record(n, trn2u_level(magnitude, negative, eight));
                    let sign = negative ^ self.negative;
                    self.negative = negative;
                    for i in 0..k - 1 {
                        let b = magnitude >> i & 1 == 1;
                        let b = self.descrambler.descramble(b);
                        self.emit(n, b);
                    }
                    let b = self.descrambler.descramble(sign);
                    self.emit(n, b);
                }
            }
            self.next += 1;
        }
    }

    /// Re-decide the 24 symbols before TRN1u's end as line history for the
    /// descrambler, then carry on from `end` as Ja.
    fn replay_to(&mut self, end: usize) {
        while self.next < end {
            let Some(z) = self.soft(self.next) else { break };
            let bit = z < 0.0;
            self.record(self.next, if bit { -1.0 } else { 1.0 });
            self.negative = bit;
            self.descrambler.descramble(bit);
            self.next += 1;
        }
    }

    fn emit(&mut self, symbol: usize, bit: bool) {
        self.bits.push(bit);
        self.bit_symbols.push(symbol);
    }

    /// Bits read since the last call: descrambled, differentially decoded.
    pub fn take_bits(&mut self) -> Vec<bool> {
        self.bit_symbols.clear();
        std::mem::take(&mut self.bits)
    }

    /// The next symbol to be decided, counted from TRN1u's first.
    pub fn symbol(&self) -> usize {
        self.next
    }

    /// TRN1u's end, when the decisions have left it.
    pub fn trn1u_end(&self) -> Option<usize> {
        self.trn1u_end
    }

    /// From symbol `from` on, read TRN2u's modulation (8.7.6). Untaken bits
    /// from `from` on are dropped and read again; those before it stay. The
    /// descrambler starts from zero as the sender's scrambler does.
    /// Su on the samples as they arrive. Su has period 6 and is half a
    /// period from itself turned over, so nearly all its power is at a
    /// sixth of the symbol rate; the last 6 samples' phasor there, turned
    /// back by the sample count, stands still while Su lasts, at any
    /// sampling phase and through any linear path. A half-symbol move turns
    /// it 30 degrees, which the reference follows; Su to S̄u turns it 180.
    /// The turn is counted in symbols from where the phasor first turned,
    /// less the channel's delay to its first strong tap.
    fn watch_su(&mut self, n: usize) {
        /// Samples of steady Su before the reference is trusted.
        const RUN: usize = 48;
        /// Samples the phasor must stay turned over.
        const HOLD: usize = 6;
        /// The cosine to the reference under which the phasor has turned:
        /// between a whole symbol's extension without a turn (0.5) and with
        /// one (-0.5).
        const TURNED: f64 = -0.25;
        if n < 6 {
            return;
        }
        let (mut re, mut im) = (0.0, 0.0);
        for m in n - 5..=n {
            let a = -std::f64::consts::TAU * (m % 6) as f64 / 6.0;
            re += self.y[m] * a.cos();
            im += self.y[m] * a.sin();
        }
        let power = (re * re + im * im) / 36.0;
        let energy: f64 = (n - 5..=n).map(|m| self.y[m] * self.y[m]).sum::<f64>() / 6.0;
        // Su's in-band power is about a third of its sample power.
        let loud = energy > FLOOR && power > 0.15 * energy;
        let (rr, ri) = self.su_ref;
        let r2 = rr * rr + ri * ri;
        if !loud {
            // A turn's own transient dips the power; only lasting quiet ends Su.
            self.su_quiet += 1;
            if self.su_quiet > 24 {
                self.su_same = 0;
                self.su_flip = None;
            }
            return;
        }
        self.su_quiet = 0;
        if self.su_same < RUN {
            // Settling: the reference follows the phasor until it is steady.
            let along = if r2 > 0.0 { (re * rr + im * ri) / (r2.sqrt() * (re * re + im * im).sqrt()) } else { 0.0 };
            self.su_same = if along > 0.95 { self.su_same + 1 } else { 0 };
            self.su_ref = (re, im);
            return;
        }
        let along = (re * rr + im * ri) / (r2.sqrt() * (re * re + im * im).sqrt());
        if along > 0.0 {
            self.su_flip = None;
            self.su_ref = (0.8 * rr + 0.2 * re, 0.8 * ri + 0.2 * im);
            return;
        }
        // S-bar-u is half a turn from Su, less whatever Jp's extension has
        // already moved it by when the modem takes the extension first: up
        // to a symbol, a sixth of a turn. So a turn is anything past a
        // quarter turn from the reference; the extension alone, at most a
        // sixth, stays on the reference's side and is followed above.
        if along > TURNED {
            return;
        }
        let first = *self.su_flip.get_or_insert((n, 0));
        self.su_flip = Some((first.0, first.1 + 1));
        if first.1 + 1 < HOLD {
            return;
        }
        self.su_flip = None;
        self.su_ref = (re, im);
        self.su_bar = !self.su_bar;
        let onset = self.channel.as_ref().map_or(0, |c| {
            let peak = c.h.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            c.h.iter().position(|v| v.abs() > 0.25 * peak).unwrap_or(0)
        });
        let symbol = first.0.saturating_sub(self.base + onset);
        self.su_turns.push(if self.su_bar { SuTurn::ToBar(symbol) } else { SuTurn::ToSu(symbol) });
    }

    /// How far behind the trained grid the Su now arriving is sent, as a
    /// fraction of a symbol in [0, 1): the phase information of 9.5.1.1.6-7.
    ///
    /// Su repeats every six symbols, so through any linear path it arrives
    /// as a 1333 Hz phasor that a delay of d symbols turns back by d sixths
    /// of a turn. The channel estimate says how that phasor would stand if
    /// Su were sent on TRN1u's symbol instants; the angle between the two is
    /// the delay. Whole symbols (and S-bar-u's half turn) fall out with the
    /// fraction. The last `window` samples must be steady Su or S-bar-u.
    pub fn su_phase(&self, window: usize) -> Option<f64> {
        let h = &self.channel.as_ref()?.h;
        let window = window - window % 6;
        if window == 0 || self.y.len() < window {
            return None;
        }
        let w = std::f64::consts::TAU / 6.0;
        let turn = |m: usize| (-(w * (m % 6) as f64)).sin_cos();
        let (mut re, mut im) = (0.0, 0.0);
        for m in self.y.len() - window..self.y.len() {
            let (sin, cos) = turn(m);
            re += self.y[m] * cos;
            im += self.y[m] * sin;
        }
        let pattern = su_pattern();
        let (mut pre, mut pim) = (0.0, 0.0);
        for m in 0..6usize {
            // One period of Su through the estimate, on the grid.
            let r: f64 = h.iter().enumerate().map(|(k, c)| c * pattern[(m + 6 * h.len() - k) % 6]).sum();
            let (sin, cos) = turn(m);
            pre += r * cos;
            pim += r * sin;
        }
        if re * re + im * im == 0.0 || pre * pre + pim * pim == 0.0 {
            return None;
        }
        // The angle of measured over predicted is minus the delay's.
        let angle = (im * pre - re * pim).atan2(re * pre + im * pim);
        Some((-angle / w).rem_euclid(1.0))
    }

    /// The delay, as a fraction of a symbol in [0, 1) from the trained grid,
    /// at which the analogue modem's symbols reach the codec most nearly as
    /// single samples: where the estimate, read between its taps, peaks.
    /// The path is band-limited under half the symbol rate, so the estimate
    /// holds it at every phase.
    pub fn best_phase(&self) -> Option<f64> {
        let h = &self.channel.as_ref()?.h;
        const STEPS: usize = 32;
        let sinc = |x: f64| if x.abs() < 1e-9 { 1.0 } else { (std::f64::consts::PI * x).sin() / (std::f64::consts::PI * x) };
        let mut best = (0.0f64, 0usize);
        for step in 0..STEPS {
            let d = step as f64 / STEPS as f64;
            let peak = (0..h.len())
                .map(|n| h.iter().enumerate().map(|(m, c)| c * sinc(n as f64 - m as f64 - d)).sum::<f64>().abs())
                .fold(0.0f64, f64::max);
            if peak > best.0 {
                best = (peak, step);
            }
        }
        (best.0 > 0.0).then(|| best.1 as f64 / STEPS as f64)
    }

    /// While the second TRN1u is being looked for: the sample the search
    /// began at, and the strongest line-up of its first signs so far with
    /// the sample it was at.
    pub fn search(&self) -> Option<(usize, f64, usize)> {
        match self.phase {
            Phase::Finding { from, .. } => Some((from, self.find_seen.0, self.find_seen.1)),
            _ => None,
        }
    }

    /// Samples received so far.
    pub fn received(&self) -> usize {
        self.y.len()
    }

    /// From now on, watch for Su and its transitions.
    pub fn listen_su(&mut self) {
        self.su_watching = true;
        self.su_same = 0;
        self.su_bar = false;
        self.su_ref = (0.0, 0.0);
        self.su_flip = None;
    }

    /// Su's transitions seen since the last call.
    pub fn take_su_turns(&mut self) -> Vec<SuTurn> {
        std::mem::take(&mut self.su_turns)
    }

    /// The second TRN1u (9.5.1.1.10) begins at or after symbol `near`, once
    /// the analogue modem has detected Jp' (9.5.2.1.9, Figure 10): find it,
    /// train again on it and count symbols from its first, as the data
    /// frames are.
    pub fn expect_trn1u(&mut self, near: usize) {
        self.su_watching = false;
        self.training = None;
        let from = (self.base + near).saturating_sub(LEAD);
        self.phase = Phase::Finding { from, best: None };
        self.find_seen = (0.0, from);
        self.next = 0;
        self.decided.clear();
        self.softs.clear();
        self.segments = vec![(0, Modulation::TwoPoint)];
        self.trn1u_end = None;
        self.trn1u = Scrambler::new(Mode::Answer);
        self.mismatches.clear();
        self.descrambler = Scrambler::new(Mode::Answer);
        self.bits.clear();
        self.bit_symbols.clear();
    }

    /// Bits read since the last call, each with the symbol it came from.
    pub fn take_bits_with_symbols(&mut self) -> Vec<(usize, bool)> {
        let symbols = std::mem::take(&mut self.bit_symbols);
        symbols.into_iter().zip(std::mem::take(&mut self.bits)).collect()
    }

    /// The sample where symbol `n` is sent, as the channel estimate counts:
    /// what reaches the codec from it is spread over the samples after.
    pub fn sample_of(&self, n: usize) -> usize {
        self.base + n
    }

    /// Received samples from `from` on, in codec units.
    pub fn samples(&self, from: usize) -> &[f64] {
        &self.y[from.min(self.y.len())..]
    }

    /// The levels decided for the symbols from `from` on, TRN1u's being 1:
    /// what the analogue modem sent, as far as the decisions are right.
    pub fn decided(&self, from: usize) -> &[f64] {
        &self.decided[from.min(self.decided.len())..]
    }

    /// Whether the receiver is trained.
    pub fn trained(&self) -> bool {
        matches!(self.phase, Phase::Trained)
    }

    pub fn switch_trn2u(&mut self, from: usize, eight: bool) {
        self.segments.retain(|(at, _)| *at < from);
        self.segments.push((from, Modulation::Trn2u { eight }));
        if self.next > from {
            let keep = self.bit_symbols.iter().take_while(|&&s| s < from).count();
            self.bits.truncate(keep);
            self.bit_symbols.truncate(keep);
            // The sign memory is the symbol before `from`.
            if let Some(&v) = from.checked_sub(1).and_then(|p| self.decided.get(p)) {
                self.negative = v < 0.0;
            }
            self.decided.truncate(from);
            self.next = from;
        }
        self.descrambler = Scrambler::new(Mode::Answer);
        self.decide();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn training_spread_over_samples_ends_where_training_at_once_did() {
        // Ru, R-bar-u, TRN1u and two-point bits, through a short channel
        // with a little noise, at codec scale.
        let mut sender = Sender::new();
        let mut tx = vec![0.0; 200];
        tx.extend(ru(384, false));
        tx.extend(ru(24, true));
        tx.extend(sender.trn1u(TRAIN + 600));
        let bits: Vec<bool> = (0..400).map(|i| (i * 7 + i / 3) % 5 < 2).collect();
        tx.extend(sender.two_point(&bits));
        tx.extend(vec![0.0; 400]);
        let h = [0.15, 1.0, -0.35, 0.12, -0.04];
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let line: Vec<f64> = (0..tx.len())
            .map(|n| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let noise = ((seed >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 60.0;
                h.iter().enumerate().filter(|&(k, _)| k <= n).map(|(k, c)| c * tx[n - k]).sum::<f64>() * 2800.0 + noise
            })
            .collect();
        let mut spread = Listener::new(crate::v90::ucode::Law::Mu, 0);
        let mut once = Listener::new(crate::v90::ucode::Law::Mu, 0);
        once.at_once = true;
        let (mut spread_heard, mut once_heard) = (Vec::new(), Vec::new());
        let (mut spread_bits, mut once_bits) = (Vec::new(), Vec::new());
        let (mut began, mut trained_at) = (None, None);
        for (i, &v) in line.iter().enumerate() {
            if let Some(h) = spread.push_linear(v) {
                if matches!(h, Heard::Trained { .. }) {
                    trained_at = Some(i);
                }
                spread_heard.push(h);
            }
            if began.is_none() && matches!(spread.phase, Phase::Training) {
                began = Some(i);
            }
            if let Some(h) = once.push_linear(v) {
                once_heard.push(h);
            }
            spread_bits.extend(spread.take_bits());
            once_bits.extend(once.take_bits());
        }
        assert_eq!(spread_heard, once_heard);
        assert!(matches!(once_heard.last(), Some(Heard::Trained { .. })), "{once_heard:?}");
        assert_eq!((&spread.w, &spread.p1, spread.delay, spread.base), (&once.w, &once.p1, once.delay, once.base));
        assert_eq!(spread.channel, once.channel);
        assert!(spread_bits.len() > 300, "{} bits", spread_bits.len());
        assert_eq!(spread_bits, once_bits);
        // Four regularisations of at most EQUALISER delays, one a sample.
        let took = trained_at.unwrap() - began.unwrap();
        assert!(took <= RATIOS.len() * (EQUALISER + 1), "trained {took} samples after TRN1u was in");
    }

    #[test]
    fn ru_and_its_reversal_are_the_table_patterns() {
        assert_eq!(ru(12, false), [1., 1., 1., -1., -1., -1., 1., 1., 1., -1., -1., -1.]);
        assert_eq!(ru(6, true), [-1., -1., -1., 1., 1., 1.]);
    }

    #[test]
    fn su_has_zeros_at_positions_1_and_4() {
        let s = su(12, false);
        let r = 1.5f64.sqrt();
        assert_eq!(s[..6], [r, 0., r, -r, 0., -r]);
        assert_eq!(su(6, true)[0], -r);
    }

    #[test]
    fn ja_s_differential_memory_starts_from_trn1u_s_last_sign() {
        // TRN1u's first five signs are all negative (GPA from zero, ones in),
        // so the memory is 1. The next scrambler output for a 1 in is
        // 1 xor out[0] = 0; 0 xor memory 1 = 1: negative.
        let mut s = Sender::new();
        assert!(s.trn1u(5).iter().all(|&v| v == -1.0));
        assert_eq!(s.two_point(&[true]), [-1.0]);
    }

    #[test]
    fn trn2u_starts_at_minus_three_over_root_five() {
        // Fresh scrambler, ones in: 1, 1. LSB 1 is magnitude 3; the sign bit 1
        // xor memory 0 is 1, negative (Table 28: 11 -> -(3/sqrt 5) LU).
        let mut s = Sender::new();
        let t = s.trn2u(12, false);
        assert_eq!(t.len(), 12);
        assert!((t[0] + 3.0 / 5f64.sqrt()).abs() < 1e-12, "{}", t[0]);
    }

    #[test]
    fn trn2u_s_sign_memory_carries_over_from_the_signal_before() {
        // Memory 1 after five negative TRN1u signs; TRN2u's first sign bit 1
        // xor 1 = 0: positive, +(3/sqrt 5).
        let mut s = Sender::new();
        s.trn1u(5);
        let t = s.trn2u(12, false);
        assert!((t[0] - 3.0 / 5f64.sqrt()).abs() < 1e-12, "{}", t[0]);
    }

    #[test]
    fn trn2u_levels_are_the_table_levels() {
        let mut s = Sender::new();
        for (eight, n) in [(false, 5f64), (true, 21f64)] {
            let allowed: Vec<f64> = (0..if eight { 4 } else { 2 }).map(|m| f64::from(2 * m + 1) / n.sqrt()).collect();
            for v in s.trn2u(600, eight) {
                assert!(allowed.iter().any(|a| (v.abs() - a).abs() < 1e-12), "{v}");
            }
        }
    }

    #[test]
    fn trn2u_bits_fill_a_short_last_symbol() {
        let mut s = Sender::new();
        assert_eq!(s.trn2u_bits(&[true; 5], false).len(), 3);
        assert_eq!(s.trn2u_bits(&[true; 7], true).len(), 3);
    }
}
