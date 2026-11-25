//! The digital modem's PCM upstream receiver: codec samples to the analogue
//! modem's data bits. Viterbi decoding over the equivalence classes, then
//! K_i, the modulus decoder and the descrambler.

use std::collections::VecDeque;

use super::modulus::ModulusDecoder;
use super::params::{ParamError, UpParams};
use super::trellis::ki;
use crate::v32::{Mode, Scrambler};
use crate::v34::trellis::{convert, label};
use crate::v90::ucode::{amplitude, from_octet, linear, Law};

/// How the indices are decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decoding {
    /// Over the convolutional code's trellis.
    Viterbi,
    /// Each sample to its nearest point, ignoring the code.
    Symbolwise,
}

/// Trellis frames kept before the oldest is decided.
const DEPTH: usize = 24;

/// One trellis frame's survivor into a state: where it came from and the
/// four indices on the way.
type Survivor = (u8, [i32; 4]);

/// Samples after the one being read that the corrector looks at: what is
/// read comes out this much later.
pub const AHEAD: usize = 40;
/// Samples before it.
const BEHIND: usize = 56;
/// The share of each miss taken out of the taps at once (normalised LMS).
const LEARN: f64 = 0.05;

/// A linear correction learnt from the receiver's own decisions.
///
/// The precoder and prefilter this end designs put each point on its codec
/// sample only as well as the channel estimate they were designed from, and
/// that estimate is short where the line's low end is long: on the ATA line
/// (2026-11-12) half the power of the data-mode miss was a broad
/// low-frequency part of the points themselves, some forty samples either
/// side, which a filter on the received samples removes (25 rms to 16 in
/// two seconds on the recorded call). The taps start at nothing and follow
/// the miss to the nearest point, which is right 99 times in 100.
#[derive(Debug, Clone)]
struct Corrector {
    taps: Vec<f64>,
    /// The latest samples, newest first; the one being read is at [`AHEAD`].
    line: VecDeque<f64>,
}

impl Corrector {
    fn new() -> Self {
        Self { taps: vec![0.0; AHEAD + BEHIND + 1], line: std::iter::repeat_n(0.0, BEHIND).collect() }
    }

    /// One sample in; the sample [`AHEAD`] before it, corrected, once there
    /// is one.
    fn push(&mut self, r: f64) -> Option<f64> {
        self.line.push_front(r);
        self.line.truncate(self.taps.len());
        (self.line.len() == self.taps.len())
            .then(|| self.line[AHEAD] - self.taps.iter().zip(&self.line).map(|(t, v)| t * v).sum::<f64>())
    }

    /// How far the last corrected sample fell from its point.
    fn learn(&mut self, miss: f64) {
        let power: f64 = self.line.iter().map(|v| v * v).sum();
        if power > 0.0 {
            let step = LEARN * miss / power;
            for (t, v) in self.taps.iter_mut().zip(&self.line) {
                *t += step * v;
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct Receiver {
    params: UpParams,
    law: Law,
    decoding: Decoding,
    /// Symbols received so far, counted from data frame interval 0.
    n: u64,
    /// Per symbol of the trellis frame being filled: the best (squared error,
    /// eta) for each residue of eta mod 4.
    residues: Vec<[(f64, i32); 4]>,
    metrics: Vec<f64>,
    history: VecDeque<Vec<Option<Survivor>>>,
    /// Decided indices waiting for a whole data frame.
    decided: Vec<i32>,
    /// Squared distance from each sample to its nearest point, summed, and
    /// how many: what the line adds to the points, as the slicer sees it.
    miss: (f64, u32),
    /// When asked for: each sample and the point nearest it, for looking at
    /// what the line added.
    trace: Option<(Vec<[f32; 3]>, super::precoder::Precoder)>,
    corrector: Option<Corrector>,
    modulus: ModulusDecoder,
    descrambler: Scrambler,
    /// Added to Ki at trellis positions 0 and 2 by the far end: 0 as 6.4.2
    /// prints it (USR, Conexant), 2 for BinModem 4's transmitter.
    pair_offset: i32,
}

impl Receiver {
    pub fn new(params: UpParams, law: Law, decoding: Decoding) -> Result<Self, ParamError> {
        if params.sets.iter().any(|c| c.max() > linear(law, 127)) {
            return Err(ParamError::BeyondCodec);
        }
        let states = params.code.states();
        let mut metrics = vec![f64::INFINITY; states];
        metrics[0] = 0.0;
        Ok(Self {
            params,
            law,
            decoding,
            n: 0,
            residues: Vec::with_capacity(4),
            metrics,
            history: VecDeque::new(),
            decided: Vec::with_capacity(12),
            miss: (0.0, 0),
            trace: None,
            corrector: None,
            modulus: ModulusDecoder::new(false),
            descrambler: Scrambler::new(Mode::Answer),
            pair_offset: 0,
        })
    }

    /// Read a far end whose pair-opening indices (trellis positions 0 and
    /// 2) sit `offset` above the class 6.4.2 prints.
    pub fn with_pair_offset(mut self, offset: i32) -> Self {
        self.pair_offset = offset;
        self
    }

    pub fn pair_offset(&self) -> i32 {
        self.pair_offset
    }

    /// Correct what the line smears across neighbouring samples, learning
    /// from the receiver's own decisions; see [`Corrector`]. What is read
    /// comes out [`AHEAD`] samples later.
    pub fn corrected(mut self) -> Self {
        self.corrector = Some(Corrector::new());
        self
    }

    /// Samples of the stream's start [`Receiver::warm`] wants.
    pub const WARM: usize = 2000;

    /// Before anything is read: learn the correction from the stream's own
    /// first samples, gone over a few times, so that they are read with it
    /// rather than while it is still being found. Half a dozen in a hundred
    /// nearest points being wrong to begin with does not stop it settling
    /// (tried on the recorded call of 2026-11-12 with its misses made 1.6
    /// times larger). Nothing without [`Receiver::corrected`], or once
    /// samples have been pushed.
    pub fn warm(&mut self, samples: &[f64]) {
        let Some(mut corrector) = self.corrector.take() else { return };
        if self.n == 0 {
            for _ in 0..4 {
                corrector.line = std::iter::repeat_n(0.0, BEHIND).collect();
                let mut n = 0usize;
                for &r in samples {
                    let Some(r) = corrector.push(r) else { continue };
                    let c = self.params.constellation(n % 12);
                    n += 1;
                    let near = (-c.n() / 2..c.n() / 2)
                        .map(|eta| f64::from(c.a(eta)))
                        .min_by(|a, b| (r - a).abs().total_cmp(&(r - b).abs()))
                        .unwrap_or(0.0);
                    corrector.learn(r - near);
                }
            }
            corrector.line = std::iter::repeat_n(0.0, BEHIND).collect();
        }
        self.corrector = Some(corrector);
    }

    /// Take the correction `other` learnt in [`Receiver::warm`], for a
    /// receiver that would warm on the very same samples: warming reads the
    /// parameters and the samples and nothing else (not the pair offset), so
    /// it comes out the same and need be done once. Both must be unread and
    /// have the same parameters; false, and nothing taken, otherwise.
    pub fn warm_from(&mut self, other: &Receiver) -> bool {
        // Unread: no symbol out and nothing pushed into the corrector, whose
        // line is left at BEHIND zeros by new() and by warm().
        let unread = |r: &Receiver| r.n == 0 && r.corrector.as_ref().is_some_and(|c| c.line.len() == BEHIND && c.line.iter().all(|&v| v == 0.0));
        if !unread(self) || !unread(other) || self.params != other.params {
            return false;
        }
        match &other.corrector {
            Some(c) => {
                self.corrector = Some(c.clone());
                true
            }
            None => false,
        }
    }

    /// One codec octet off the digital network.
    pub fn push_octet(&mut self, octet: u8) -> Vec<bool> {
        let (ucode, negative) = from_octet(self.law, octet);
        self.push_linear(f64::from(amplitude(self.law, ucode, negative)))
    }

    /// One sample in the constellations' linear scale.
    pub fn push_linear(&mut self, r: f64) -> Vec<bool> {
        let r = match self.corrector.as_mut() {
            None => r,
            Some(c) => match c.push(r) {
                Some(r) => r,
                None => return Vec::new(),
            },
        };
        let interval = (self.n % 12) as usize;
        self.n += 1;
        let c = self.params.constellation(interval);
        let mut best = [(f64::INFINITY, 0); 4];
        for eta in -c.n() / 2..c.n() / 2 {
            let e = r - f64::from(c.a(eta));
            let slot = &mut best[eta.rem_euclid(4) as usize];
            if e * e < slot.0 {
                *slot = (e * e, eta);
            }
        }
        let near = best.iter().min_by(|a, b| a.0.total_cmp(&b.0)).map_or(0, |b| b.1);
        self.miss.0 += best[near.rem_euclid(4) as usize].0;
        self.miss.1 += 1;
        if let Some(corrector) = self.corrector.as_mut() {
            corrector.learn(r - f64::from(c.a(near)));
        }
        if let Some((t, precoder)) = self.trace.as_mut() {
            // What the analogue modem sent, were the nearest point the one.
            let a = f64::from(c.a(near));
            t.push([r as f32, a as f32, precoder.send(a) as f32]);
        }
        match self.decoding {
            Decoding::Symbolwise => {
                let eta = best.iter().min_by(|a, b| a.0.total_cmp(&b.0)).map_or(0, |b| b.1);
                self.decide(eta)
            }
            Decoding::Viterbi => {
                self.residues.push(best);
                if self.residues.len() < 4 {
                    return Vec::new();
                }
                self.step();
                if self.history.len() > DEPTH {
                    let etas = self.traceback(1)[0];
                    self.history.pop_front();
                    etas.into_iter().flat_map(|e| self.decide(e)).collect()
                } else {
                    Vec::new()
                }
            }
        }
    }

    /// RMS distance from the samples since the last call to their nearest
    /// points, and how many samples that was over.
    pub fn take_miss(&mut self) -> Option<(f64, u32)> {
        let (sum, n) = std::mem::replace(&mut self.miss, (0.0, 0));
        (n > 0).then(|| ((sum / f64::from(n)).sqrt(), n))
    }

    /// Keep each sample and its nearest point from here on (a diagnostic).
    pub fn trace(&mut self) {
        if self.trace.is_none() {
            let precoder = super::precoder::Precoder::new(self.params.filters.clone(), self.params.gain);
            self.trace = Some((Vec::new(), precoder));
        }
    }

    /// Kept since the last call: each sample, the point nearest it, and the
    /// transmitter's sample for that point.
    pub fn take_trace(&mut self) -> Vec<[f32; 3]> {
        self.trace.as_mut().map(|t| std::mem::take(&mut t.0)).unwrap_or_default()
    }

    /// The end of the stream: decide everything still held.
    pub fn finish(&mut self) -> Vec<bool> {
        // The corrector still holds its last samples: push them through.
        let mut held = Vec::new();
        if self.corrector.is_some() {
            for _ in 0..AHEAD {
                held.extend(self.push_linear(0.0));
            }
            self.corrector = None;
        }
        held.extend(self.finish_decoder());
        held
    }

    fn finish_decoder(&mut self) -> Vec<bool> {
        let held = self.history.len();
        let frames = self.traceback(held);
        self.history.clear();
        frames.into_iter().flatten().flat_map(|e| self.decide(e)).collect()
    }

    /// Add-compare-select over one trellis frame.
    fn step(&mut self) {
        let r = std::mem::take(&mut self.residues);
        // Per pair: the best (cost, eta, eta) for each label and parity.
        let pair = |a: &[(f64, i32); 4], b: &[(f64, i32); 4]| {
            let mut out = [[(f64::INFINITY, 0, 0); 2]; 8];
            for (ra, &(ca, ea)) in a.iter().enumerate() {
                for (rb, &(cb, eb)) in b.iter().enumerate() {
                    let l = label((2 * ra as i32 + 1, 2 * rb as i32 + 1)) as usize;
                    let parity = (ra + rb) % 2;
                    if ca + cb < out[l][parity].0 {
                        out[l][parity] = (ca + cb, ea, eb);
                    }
                }
            }
            out
        };
        let (first, second) = (pair(&r[0], &r[1]), pair(&r[2], &r[3]));
        let code = self.params.code;
        let mut next = vec![f64::INFINITY; code.states()];
        let mut survivors: Vec<Option<Survivor>> = vec![None; code.states()];
        for s in 0..code.states() {
            let m = self.metrics[s];
            if !m.is_finite() {
                continue;
            }
            let y0 = usize::from(code.output(s as u8));
            for (la, fa) in first.iter().enumerate() {
                for (pa, &(ca, e0, e1)) in fa.iter().enumerate() {
                    if !ca.is_finite() {
                        continue;
                    }
                    for (lb, fb) in second.iter().enumerate() {
                        let (cb, e2, e3) = fb[(y0 + pa) % 2];
                        let cost = m + ca + cb;
                        if !cost.is_finite() {
                            continue;
                        }
                        let ns = usize::from(code.next(s as u8, convert(la as u8, lb as u8)));
                        if cost < next[ns] {
                            next[ns] = cost;
                            survivors[ns] = Some((s as u8, [e0, e1, e2, e3]));
                        }
                    }
                }
            }
        }
        // Keep the numbers small; only differences matter.
        let low = next.iter().copied().fold(f64::INFINITY, f64::min);
        if low.is_finite() {
            next.iter_mut().for_each(|m| *m -= low);
        }
        self.metrics = next;
        self.history.push_back(survivors);
    }

    /// The indices of the oldest `count` trellis frames held, traced back
    /// from the best state now.
    fn traceback(&self, count: usize) -> Vec<[i32; 4]> {
        let mut state = (0..self.metrics.len())
            .min_by(|&a, &b| self.metrics[a].total_cmp(&self.metrics[b]))
            .unwrap_or(0);
        let mut path = Vec::with_capacity(self.history.len());
        for survivors in self.history.iter().rev() {
            let (prev, etas) = survivors[state].unwrap_or((0, [0; 4]));
            path.push(etas);
            state = usize::from(prev);
        }
        path.reverse();
        path.truncate(count);
        path
    }

    /// One decided index; bits come out at the end of each data frame.
    fn decide(&mut self, eta: i32) -> Vec<bool> {
        self.decided.push(eta);
        if self.decided.len() < 12 {
            return Vec::new();
        }
        let ks: [u16; 12] =
            std::array::from_fn(|i| {
                let eta = self.decided[i];
                let eta = if i % 4 == 0 || i % 4 == 2 { eta - self.pair_offset } else { eta };
                ki(i % 4, eta, self.params.moduli[i])
            });
        self.decided.clear();
        let bits = self.modulus.decode(&ks, &self.params.moduli, self.params.k);
        bits.into_iter().map(|b| self.descrambler.descramble(b)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v90::ucode::Law;
    use crate::v92::upstream::fixture::{frames, params};
    use crate::v92::upstream::params::{Constellation, ParamError};
    use crate::v92::upstream::transmitter::{Sent, Transmitter};

    fn send(n: usize) -> (Vec<Vec<bool>>, Vec<Sent>) {
        let data = frames(n, 48, 11);
        let mut tx = Transmitter::new(params());
        let sent = data.iter().map(|f| tx.frame(f).unwrap()).collect();
        (data, sent)
    }

    fn receive(decoding: Decoding, samples: impl Iterator<Item = f64>) -> Vec<bool> {
        let mut rx = Receiver::new(params(), Law::Mu, decoding).unwrap();
        let mut bits: Vec<bool> = samples.flat_map(|r| rx.push_linear(r)).collect();
        bits.extend(rx.finish());
        bits
    }

    #[test]
    fn points_beyond_the_codec_are_refused() {
        let mut p = params();
        p.sets = vec![Constellation::from_positive(&[100, 32125])];
        assert_eq!(Receiver::new(p, Law::Mu, Decoding::Viterbi).err(), Some(ParamError::BeyondCodec));
    }

    #[test]
    fn clean_points_decode_exactly() {
        let (data, sent) = send(200);
        for decoding in [Decoding::Viterbi, Decoding::Symbolwise] {
            let got = receive(decoding, sent.iter().flat_map(|s| s.samples));
            assert_eq!(got, data.concat(), "{decoding:?}");
        }
    }

    /// BinModem 4 (PAIR_FIRST_INDEX_OFFSET, from an Agere SV92PP capture)
    /// sends the first index of each 4D pair two above the class 6.4.2
    /// prints. The pair's subset label is unchanged by it, so the trellis
    /// still decodes; only Ki comes back wrong unless the receiver is told.
    #[test]
    fn a_transmitter_with_its_pair_indices_two_up_reads_with_that_offset() {
        let (data, sent) = send(200);
        let p = params();
        let mut levels = Vec::new();
        for s in &sent {
            for i in 0..12 {
                let c = p.constellation(i);
                let eta = if i % 4 == 0 || i % 4 == 2 { s.etas[i] + 2 } else { s.etas[i] };
                assert!(eta < c.n() / 2, "the fixture's indices leave room for the shift");
                levels.push(f64::from(c.a(eta)));
            }
        }
        let read = |offset: i32| {
            let mut rx = Receiver::new(params(), Law::Mu, Decoding::Viterbi).unwrap().with_pair_offset(offset);
            let mut bits: Vec<bool> = levels.iter().flat_map(|&r| rx.push_linear(r)).collect();
            bits.extend(rx.finish());
            bits
        };
        assert_eq!(read(2), data.concat());
        assert_ne!(read(0), data.concat());
    }

    #[test]
    fn a_stream_shorter_than_the_traceback_still_decodes() {
        let (data, sent) = send(2);
        assert_eq!(receive(Decoding::Viterbi, sent.iter().flat_map(|s| s.samples)), data.concat());
    }

    /// What the ATA line of 2026-11-12 did to data: a broad low-frequency
    /// part of the points themselves taken away (the line passes no DC and
    /// the precoder's filters do not quite make up for it), spread over
    /// some forty samples either side; sized here, as it was there, to put
    /// the samples a good part of the way to the nearest wrong point.
    fn smeared(points: &[f64]) -> Vec<f64> {
        let n = points.len() as i64;
        (0..n)
            .map(|i| {
                let near: f64 = (-30i64..=40)
                    .filter(|&k| i - k >= 0 && i - k < n)
                    .map(|k| {
                        let x = (k as f64 - 5.0) / 35.0;
                        0.001 * (1.0 - x * x).max(0.0) * points[(i - k) as usize]
                    })
                    .sum();
                points[i as usize] - near
            })
            .collect()
    }

    #[test]
    fn a_low_frequency_smear_is_learnt_and_taken_out() {
        let (data, sent) = send(3000);
        let line = smeared(&sent.iter().flat_map(|s| s.samples).collect::<Vec<f64>>());
        let miss = |mut rx: Receiver| {
            let mut bits = Vec::new();
            for &r in &line[..24_000] {
                bits.extend(rx.push_linear(r));
            }
            rx.take_miss();
            for &r in &line[24_000..] {
                bits.extend(rx.push_linear(r));
            }
            let late = rx.take_miss().unwrap().0;
            bits.extend(rx.finish());
            (late, bits)
        };
        let (plain, _) = miss(Receiver::new(params(), Law::Mu, Decoding::Viterbi).unwrap());
        let (corrected, bits) = miss(Receiver::new(params(), Law::Mu, Decoding::Viterbi).unwrap().corrected());
        assert!(corrected < plain / 3.0, "{corrected} against {plain} without");
        // Once learnt, every frame reads; and none is lost at either end.
        let all = data.concat();
        assert_eq!(bits.len(), all.len());
        assert_eq!(bits[all.len() / 2..], all[all.len() / 2..]);
    }

    #[test]
    fn warmed_on_its_own_first_samples_the_start_reads_too() {
        let (data, sent) = send(1000);
        let line = smeared(&sent.iter().flat_map(|s| s.samples).collect::<Vec<f64>>());
        // How far the first quarter second falls from its points, and all
        // that is read.
        let read = |mut rx: Receiver| {
            let mut bits: Vec<bool> = line[..2000].iter().flat_map(|&r| rx.push_linear(r)).collect();
            let early = rx.take_miss().unwrap().0;
            bits.extend(line[2000..].iter().flat_map(|&r| rx.push_linear(r)));
            bits.extend(rx.finish());
            (early, bits)
        };
        let (cold, _) = read(Receiver::new(params(), Law::Mu, Decoding::Viterbi).unwrap().corrected());
        let mut rx = Receiver::new(params(), Law::Mu, Decoding::Viterbi).unwrap().corrected();
        rx.warm(&line[..Receiver::WARM]);
        let (warm, bits) = read(rx);
        assert!(warm < cold / 2.0, "{warm} against {cold} cold");
        assert_eq!(bits, data.concat());
    }

    #[test]
    fn a_warm_up_taken_from_another_reads_as_its_own_does() {
        let (data, sent) = send(600);
        let line = smeared(&sent.iter().flat_map(|s| s.samples).collect::<Vec<f64>>());
        let read = |mut rx: Receiver| {
            let mut bits: Vec<bool> = line.iter().flat_map(|&r| rx.push_linear(r)).collect();
            bits.extend(rx.finish());
            (bits, rx.take_miss())
        };
        let fresh = || Receiver::new(params(), Law::Mu, Decoding::Viterbi).unwrap().corrected();
        let mut own = fresh();
        own.warm(&line[..Receiver::WARM]);
        // The other pair offset's receiver at the same position.
        let mut taker = fresh().with_pair_offset(2);
        assert!(taker.warm_from(&own));
        let mut taker_own = fresh().with_pair_offset(2);
        taker_own.warm(&line[..Receiver::WARM]);
        assert_eq!(read(taker), read(taker_own));
        let (bits, _) = read(own.clone());
        assert_eq!(bits, data.concat());
        // Nothing taken from one already reading, or into one without a
        // corrector, or across parameters.
        let mut reading = own.clone();
        let _ = reading.push_linear(line[0]);
        let mut late = fresh();
        assert!(!late.warm_from(&reading));
        let mut plain = Receiver::new(params(), Law::Mu, Decoding::Viterbi).unwrap();
        assert!(!plain.warm_from(&own));
    }

    #[test]
    fn clean_points_still_decode_exactly_when_corrected() {
        let (data, sent) = send(200);
        let mut rx = Receiver::new(params(), Law::Mu, Decoding::Viterbi).unwrap().corrected();
        let mut bits: Vec<bool> = sent.iter().flat_map(|s| s.samples).flat_map(|r| rx.push_linear(r)).collect();
        bits.extend(rx.finish());
        assert_eq!(bits, data.concat());
    }

    #[test]
    fn viterbi_repairs_a_symbol_the_slicer_gets_wrong() {
        let (data, sent) = send(40);
        let p = params();
        let c = p.constellation(3);
        // A trellis-position-3 interval whose neighbour of the other parity exists.
        let (interval, eta) = [3, 7, 11]
            .into_iter()
            .map(|i| (i, sent[20].etas[i]))
            .find(|&(_, e)| { let n = if e % 2 == 0 { e - 1 } else { e + 1 }; n >= -c.n() / 2 && n < c.n() / 2 })
            .unwrap();
        let other = if eta % 2 == 0 { eta - 1 } else { eta + 1 };
        let mut samples: Vec<f64> = sent.iter().flat_map(|s| s.samples).collect();
        let (a, b) = (f64::from(c.a(eta)), f64::from(c.a(other)));
        samples[20 * 12 + interval] = a + 0.6 * (b - a);

        let k = 48;
        let slicer = receive(Decoding::Symbolwise, samples.iter().copied());
        assert_ne!(slicer[20 * k..21 * k], data[20][..], "the slicer should have been fooled");
        assert_eq!(receive(Decoding::Viterbi, samples.into_iter()), data.concat());
    }
}
