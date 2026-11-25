//! Step 4: the digital modem's design of the analogue modem's PCM
//! transmitter, from what TRN1u brought (V.92 8.5.7, 6.4, Table 30).

use crate::v32::{Mode, Scrambler};
use crate::v90::ucode::{self, Law};
use crate::v92::phase4::{Cpd, Filters, UpConstellations, UpTrellis};
use crate::v92::PcmUpstreamRequest;

use super::params::UpParams;
use super::transmitter::Transmitter;

/// TRN1u's signs: the GPA scrambler from zero, fed ones (8.5.7). "A
/// scrambler output of 0 represents a positive voltage; a scrambler output
/// of 1 represents a negative voltage": here +1 and -1.
pub fn trn1u_signs(n: usize) -> Vec<f64> {
    let mut scrambler = Scrambler::new(Mode::Answer);
    (0..n).map(|_| if scrambler.scramble(true) { -1.0 } else { 1.0 }).collect()
}

/// Solve `a x = b` by Gaussian elimination with partial pivoting. `a` is
/// square and small here (a few dozen); None where it is singular.
pub(crate) fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let f = a[row][col] / a[col][col];
            if f != 0.0 {
                for k in col..n {
                    a[row][k] -= f * a[col][k];
                }
                b[row] -= f * b[col];
            }
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let tail: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - tail) / a[row][row];
    }
    Some(x)
}

/// The path from the analogue modem's transmitter to the codec, in codec
/// linear units per transmit unit (a TRN1u symbol is one unit), and the
/// noise the codec adds on top, as a variance.
#[derive(Debug, Clone, PartialEq)]
pub struct Channel {
    pub h: Vec<f64>,
    pub noise: f64,
}

/// Least squares: the `taps`-long causal response that best turns `signs`
/// into `received`, and what is left over.
///
/// Only samples with a full history count, from `taps - 1` on.
pub fn estimate(received: &[f64], signs: &[f64], taps: usize) -> Channel {
    let len = received.len().min(signs.len());
    let rows = (taps.saturating_sub(1)..len).collect::<Vec<_>>();
    let (ata, atb) = normal_equations(received, signs, taps, len);
    let h = solve(ata, atb).unwrap_or_else(|| vec![0.0; taps]);
    let residual: f64 = rows
        .iter()
        .map(|&n| {
            let y: f64 = (0..taps).map(|k| h[k] * signs[n - k]).sum();
            (received[n] - y).powi(2)
        })
        .sum();
    let noise = if rows.len() > taps { residual / (rows.len() - taps) as f64 } else { 0.0 };
    Channel { h, noise }
}

/// [`estimate`]'s normal equations over rows `taps - 1 .. len`:
/// A'A[i][j] = sum_n s[n-i] s[n-j] and A'b[i] = sum_n s[n-i] r[n].
///
/// A'A is the same sum shifted down a diagonal, so each entry is the one
/// above-left of it plus the row that comes in at the start and less the
/// one that goes out at the end: its first row and column are summed
/// outright and the rest follow in O(taps^2), not O(len taps^2). That was
/// 60-90 ms of one sample on the server for TRN2u's 4096 x 80, a stall the
/// line heard.
fn normal_equations(received: &[f64], signs: &[f64], taps: usize, len: usize) -> (Vec<Vec<f64>>, Vec<f64>) {
    let mut ata = vec![vec![0.0; taps]; taps];
    let mut atb = vec![0.0; taps];
    if taps == 0 || len < taps {
        return (ata, atb);
    }
    let first = taps - 1;
    for i in 0..taps {
        atb[i] = (first..len).map(|n| signs[n - i] * received[n]).sum();
        let edge: f64 = (first..len).map(|n| signs[n] * signs[n - i]).sum();
        ata[0][i] = edge;
        ata[i][0] = edge;
    }
    for i in 1..taps {
        for j in i..taps {
            // Rows first..len shifted by one: n = first - 1 comes in, len - 1 goes out.
            let v = ata[i - 1][j - 1] + signs[first - i] * signs[first - j] - signs[len - i] * signs[len - j];
            ata[i][j] = v;
            ata[j][i] = v;
        }
    }
    (ata, atb)
}

/// With our echo on the line: the channel's noise without the codec's
/// rounding, which [`design`] then puts back point by point, at the step
/// where each point lies.
///
/// What was heard is signal plus echo rounded, less the echo: the rounding
/// is spread evenly over the codec's cell there, a twelfth of the step
/// squared, and the step is the one at the sample's own size, the echo
/// being small beside it. The data-mode samples of 2026-11-12 fit this
/// (12.5 rms at the bottom, 22 from 991, 34 from 2015). Never under half
/// the measured figure: the model is not to be trusted further than that.
pub fn unrounded(channel: &Channel, heard: &[f64], law: Law) -> Channel {
    let used = &heard[channel.h.len().saturating_sub(1).min(heard.len())..];
    let rounding = if used.is_empty() {
        0.0
    } else {
        used.iter().map(|v| codec_step(law, v.abs()).powi(2) / 12.0).sum::<f64>() / used.len() as f64
    };
    Channel { h: channel.h.clone(), noise: (channel.noise - rounding).max(channel.noise / 4.0) }
}

/// The channel's noise without what the codec added quantising TRN1u.
///
/// TRN1u's samples fall anywhere between the codec's levels, so
/// [`estimate`]'s residual carries the codec's rounding; data does not,
/// since its points are the codec's levels and arrive as themselves. Nor can
/// the rounding simply be subtracted: with noise smaller than a step it is
/// not independent of the signal. So the noise is fitted directly, by
/// maximum likelihood: each heard level is a cell of the codec, the channel
/// predicts where in it the sample was aimed, and sigma is whatever makes the
/// cells actually hit most likely. `heard` and `signs` are what [`estimate`]
/// was given.
pub fn analogue(channel: &Channel, heard: &[f64], signs: &[f64], law: Law) -> Channel {
    let mut values: Vec<f64> = (0..=127u8)
        .flat_map(|u| {
            let v = f64::from(ucode::linear(law, u));
            [v, -v]
        })
        .collect();
    values.sort_by(f64::total_cmp);
    values.dedup();
    let taps = channel.h.len();
    let len = heard.len().min(signs.len());
    // (lower bound - prediction, upper bound - prediction) per sample.
    let gaps: Vec<(f64, f64)> = (taps.saturating_sub(1)..len)
        .map(|n| {
            let predicted: f64 = (0..taps).map(|k| channel.h[k] * signs[n - k]).sum();
            let i = values.partition_point(|&v| v < heard[n]).min(values.len() - 1);
            let lower = if i == 0 { f64::NEG_INFINITY } else { (values[i - 1] + values[i]) / 2.0 };
            let upper = if i + 1 == values.len() { f64::INFINITY } else { (values[i] + values[i + 1]) / 2.0 };
            (lower - predicted, upper - predicted)
        })
        .collect();
    let likelihood = |sigma: f64| -> f64 {
        gaps.iter().map(|&(a, b)| (phi(b / sigma) - phi(a / sigma)).max(1e-300).ln()).sum()
    };
    // Golden section over log sigma.
    let (mut lo, mut hi) = (0.25f64.ln(), (4.0 * channel.noise.sqrt() + 1.0).ln());
    let r = (5f64.sqrt() - 1.0) / 2.0;
    for _ in 0..60 {
        let (m1, m2) = (hi - r * (hi - lo), lo + r * (hi - lo));
        if likelihood(m1.exp()) < likelihood(m2.exp()) {
            lo = m1;
        } else {
            hi = m2;
        }
    }
    let sigma = ((lo + hi) / 2.0).exp();
    Channel { h: channel.h.clone(), noise: sigma * sigma }
}

/// The standard normal's distribution function.
fn phi(x: f64) -> f64 {
    if x == f64::INFINITY {
        return 1.0;
    }
    if x == f64::NEG_INFINITY {
        return 0.0;
    }
    0.5 * erfc(-x / std::f64::consts::SQRT_2)
}

/// Complementary error function: Numerical Recipes' erfcc, |error| < 1.2e-7.
fn erfc(x: f64) -> f64 {
    let z = x.abs();
    let t = 1.0 / (1.0 + 0.5 * z);
    let r = t * (-z * z - 1.26551223
        + t * (1.00002368
            + t * (0.37409196
                + t * (0.09678418
                    + t * (-0.18628806
                        + t * (0.27886807 + t * (-1.13520398 + t * (1.48851587 + t * (-0.82215223 + t * 0.17087277)))))))))
        .exp();
    if x >= 0.0 { r } else { 2.0 - r }
}

/// A prefilter `f` (z2 before scaling) and the precoder feedback `p1` that
/// together make the codec see the chosen point, `delay` samples late.
#[derive(Debug, Clone, PartialEq)]
pub struct Equaliser {
    pub f: Vec<f64>,
    pub p1: Vec<f64>,
    pub delay: usize,
    /// Energy of h*f outside the main tap and the feedback's reach, with
    /// the main tap at one: the ISI nothing cancels, per unit of x power.
    pub residual: f64,
}

/// Design `f` (`lf` taps) and `p1` (`lp` taps) for `h`.
///
/// For each delay, `f` minimises the ISI outside the window [delay,
/// delay + lp] plus mu |f|^2, with (h*f)[delay] = 1: f = R^-1 g / g'R^-1 g.
/// The feedback then cancels what falls inside the window, so it is
/// -(h*f)[delay + k]. Delays whose feedback does not fit Q1.14 are passed
/// over unless none fits; of the rest the least residual + mu|f|^2 wins.
pub fn equalise(h: &[f64], lf: usize, lp: usize) -> Equaliser {
    equalise_with(h, lf, lp, 1e-3)
}

/// [`equalise`] with mu = `ratio` |h|^2: smaller buys less ISI with more
/// transmit power spent on the prefilter.
pub fn equalise_with(h: &[f64], lf: usize, lp: usize, ratio: f64) -> Equaliser {
    let mut job = EqualiseJob::new(h, lf, lp, ratio);
    loop {
        if let Some(e) = job.step(usize::MAX) {
            return e;
        }
    }
}

/// [`equalise_with`] a few delays at a time, so a caller on the sample path
/// can spread its 64-odd solves over as many samples rather than stall one
/// (each delay is one solve of an `lf`-square system). Run to the end it
/// gives exactly what [`equalise_with`] gives.
#[derive(Debug, Clone)]
pub struct EqualiseJob {
    h: Vec<f64>,
    lf: usize,
    lp: usize,
    rows: usize,
    mu: f64,
    /// Row n of the convolution matrix: (h*f)[n] = sum_j h[n-j] f[j].
    a: Vec<Vec<f64>>,
    r: Vec<Vec<f64>>,
    delay: usize,
    best: Option<(bool, f64, Equaliser)>,
    done: Option<Equaliser>,
}

impl EqualiseJob {
    pub fn new(h: &[f64], lf: usize, lp: usize, ratio: f64) -> Self {
        let rows = (h.len() + lf).saturating_sub(1);
        let a: Vec<Vec<f64>> = (0..rows)
            .map(|n| (0..lf).map(|j| if n >= j && n - j < h.len() { h[n - j] } else { 0.0 }).collect())
            .collect();
        let mu = ratio * h.iter().map(|v| v * v).sum::<f64>();
        let mut job = Self { h: h.to_vec(), lf, lp, rows, mu, a, r: Vec::new(), delay: 0, best: None, done: None };
        if h.is_empty() || lf == 0 {
            job.done = Some(job.none());
        } else {
            job.r = job.fresh(0);
        }
        job
    }

    fn none(&self) -> Equaliser {
        Equaliser { f: vec![0.0; self.lf], p1: vec![0.0; self.lp], delay: 0, residual: f64::INFINITY }
    }

    fn outside(&self, delay: usize, n: usize) -> bool {
        n < delay || n > delay + self.lp
    }

    fn add(r: &mut [Vec<f64>], row: &[f64], sign: f64) {
        let lf = row.len();
        for i in 0..lf {
            if row[i] == 0.0 {
                continue;
            }
            for j in 0..lf {
                r[i][j] += sign * row[i] * row[j];
            }
        }
    }

    /// R for one delay, from nothing: mu I plus the rows outside the window.
    fn fresh(&self, delay: usize) -> Vec<Vec<f64>> {
        let mut r = vec![vec![0.0; self.lf]; self.lf];
        for (i, ri) in r.iter_mut().enumerate() {
            ri[i] = self.mu;
        }
        for n in (0..self.rows).filter(|&n| self.outside(delay, n)) {
            Self::add(&mut r, &self.a[n], 1.0);
        }
        r
    }

    /// Up to `delays` more delays tried; the equaliser once all have been.
    pub fn step(&mut self, delays: usize) -> Option<Equaliser> {
        let (lf, lp, rows) = (self.lf, self.lp, self.rows);
        let last = lf.min(rows);
        let mut left = delays;
        while self.done.is_none() && left > 0 {
            if self.delay >= last {
                let best = self.best.take().map(|(_, _, e)| e);
                self.done = Some(best.unwrap_or_else(|| self.none()));
                break;
            }
            left -= 1;
            let delay = self.delay;
            self.delay += 1;
            if delay > 0 {
                // Slide the window by one: row delay-1 leaves it, row
                // delay+lp enters it. Rebuilt every few delays against drift.
                if delay % 8 == 0 {
                    self.r = self.fresh(delay);
                } else {
                    Self::add(&mut self.r, &self.a[delay - 1], 1.0);
                    if delay + lp < rows {
                        Self::add(&mut self.r, &self.a[delay + lp], -1.0);
                    }
                }
            }
            let g = self.a[delay].clone();
            let Some(rg) = solve(self.r.clone(), g.clone()) else { continue };
            let norm: f64 = g.iter().zip(&rg).map(|(a, b)| a * b).sum();
            if norm.abs() < 1e-300 {
                continue;
            }
            let f: Vec<f64> = rg.iter().map(|v| v / norm).collect();
            let mut c = vec![0.0; rows];
            for (k, hk) in self.h.iter().enumerate() {
                for (j, fj) in f.iter().enumerate() {
                    c[k + j] += hk * fj;
                }
            }
            let p1: Vec<f64> = (1..=lp).map(|k| -c.get(delay + k).copied().unwrap_or(0.0)).collect();
            let residual: f64 = c.iter().enumerate().filter(|&(n, _)| self.outside(delay, n)).map(|(_, v)| v * v).sum();
            let fits = p1.iter().all(|p| p.abs() < 2.0 - 1.0 / 16384.0);
            let cost = residual + self.mu * f.iter().map(|v| v * v).sum::<f64>();
            let better = match &self.best {
                None => true,
                // A later delay is latency; it has to be clearly better.
                Some((best_fits, best_cost, _)) => {
                    (fits && !best_fits) || (fits == *best_fits && cost < best_cost * 0.99)
                }
            };
            if better {
                self.best = Some((fits, cost, Equaliser { f, p1, delay, residual }));
            }
        }
        self.done.clone()
    }
}

/// What INFO1a said the analogue modem's filters can be (Table 18).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limits {
    pub sections: u8,
    pub ltot: u16,
    pub lmax: u16,
}

impl Limits {
    /// Whether `cpd`'s filters keep to these limits: Ltot over all four,
    /// Lmax over each, z1 only with sections bit 0 and p2 only with bit 1
    /// (Table 18; the same rule as [`Cpd::fits`]).
    pub fn admits(&self, cpd: &Cpd) -> bool {
        let Some(filters) = &cpd.filters else { return true };
        let [z1, p1, z2, p2] = filters.lengths();
        z1 + p1 + z2 + p2 <= usize::from(self.ltot)
            && [z1, p1, z2, p2].iter().all(|&l| l <= usize::from(self.lmax))
            && (z1 == 0 || self.sections & 1 == 1)
            && (p2 == 0 || self.sections & 2 == 2)
    }
}

impl From<&PcmUpstreamRequest> for Limits {
    fn from(r: &PcmUpstreamRequest) -> Self {
        Self { sections: r.filter_sections, ltot: r.total_coefficient_limit, lmax: r.section_coefficient_limit }
    }
}

/// What the design is aiming at.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Target {
    pub law: Law,
    /// Mean square allowed at the transmitter's output, TRN1u's being 1.
    pub power: f64,
    /// Extra spacing between points beyond the 7-sigma baseline, in dB.
    pub margin_db: f64,
    pub limits: Limits,
    /// This end's echo reaches the codec with the signal. The codec then
    /// rounds point plus echo, not the point: its rounding is noise spread
    /// over the cell, growing with the codec's step, and the points are
    /// spaced for it where they lie.
    pub echo: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DesignError {
    /// The prefilter needs a gain G of a quarter or more, past gain4's range.
    TooLoud,
    /// Not even the lowest rate has room between its points for the noise.
    TooNoisy,
    /// Nothing that fits: no feedback the precoder can carry (Q1.14)
    /// equalises this channel, or every candidate failed its rehearsal
    /// (power, crest) or INFO1a's filter limits.
    NoFit,
}

/// A designed CPd, and how many samples late the codec sees each symbol.
#[derive(Debug, Clone, PartialEq)]
pub struct Design {
    pub cpd: Cpd,
    pub delay: usize,
}

/// Design the analogue modem's transmitter for `channel`.
///
/// The model: z2 = f/G and p1 from [`equalise`] make the codec sample
/// `delay` late equal the precoder's chosen point plus the ISI left over, and
/// the transmitted sample is f*x, whose mean square is about |f|^2 E[x^2].
/// The precoder keeps x within the constellation, roughly uniform, so
/// E[x^2] is about A^2/3 for a largest point A: hence the largest point the
/// power allows. Points are then picked from the codec's own levels, spaced
/// for the noise and the leftover ISI, and the moduli and rate follow.
///
/// The prefilter's power regulariser is tried at several strengths and the
/// best rate kept: too strong leaves ISI that grows with the signal, too
/// weak spends the power budget on the prefilter.
pub fn design(channel: &Channel, target: &Target) -> Result<Design, DesignError> {
    let limits = target.limits;
    let len = usize::from(limits.lmax).min(usize::from(limits.ltot) / 2).min(96);
    let mut best: Result<(Design, f64), DesignError> = Err(DesignError::NoFit);
    for ratio in [1e-2, 1e-3, 1e-4, 1e-5, 1e-6] {
        let e = equalise_with(&channel.h, len, len, ratio);
        match build(channel, target, e) {
            Ok((d, spacing)) => {
                let better = match &best {
                    Ok((b, b_spacing)) => (d.cpd.drn, -spacing) > (b.cpd.drn, -b_spacing),
                    Err(_) => true,
                };
                if better {
                    best = Ok((d, spacing));
                }
            }
            // Keep the most telling error: TooNoisy over TooLoud over NoFit.
            Err(err) => {
                if let Err(prev) = &best {
                    let rank = |e: &DesignError| match e {
                        DesignError::TooNoisy => 2,
                        DesignError::TooLoud => 1,
                        DesignError::NoFit => 0,
                    };
                    if rank(&err) > rank(prev) {
                        best = Err(err);
                    }
                }
            }
        }
    }
    best.map(|(d, _)| d)
}

/// Point spacing in standard deviations of noise plus leftover ISI, before
/// the margin. Half a spacing of 3.5 sigma is what an uncoded slicer wants;
/// the 16-state trellis code the receiver decodes (6.4.4, V.34's) is worth
/// about 4 dB, of which 1.3 are taken here. At 5 the scripted start-up's B1u
/// no longer reads.
const SIGMAS: f64 = 6.0;

/// The rest of [`design`] for one equaliser; the spacing chosen comes back
/// too, to break ties between equal rates.
///
/// Every candidate is rehearsed: the real transmitter, quantised
/// coefficients and all, over the estimated channel without noise. Its
/// actual power and crest decide whether it is kept, and its actual leftover
/// ISI widens the spacing it is re-tried at. Power is not monotonic in the
/// largest point -- the moduli and the precoder's choices move under it --
/// so the largest point is searched over a grid from the codec's top down
/// rather than steered, and the best rate kept.
fn build(channel: &Channel, target: &Target, e: Equaliser) -> Result<(Design, f64), DesignError> {
    if !e.p1.iter().all(|p| p.abs() < 2.0 - 1.0 / 16384.0) || !e.residual.is_finite() {
        return Err(DesignError::NoFit);
    }

    // G as small as z2's Q0.15 allows, on gain4's grid (4G in Q0.16).
    let peak = e.f.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let gain4 = (4.0 * peak * (32768.0 / 32767.0) * 65536.0).ceil();
    if gain4 > 65535.0 || peak == 0.0 {
        return Err(DesignError::TooLoud);
    }
    let g = gain4 / 262_144.0;
    let z2: Vec<i16> = e.f.iter().map(|v| (v / g * 32768.0).round().clamp(-32768.0, 32767.0) as i16).collect();
    let p1: Vec<i16> = e.p1.iter().map(|v| (v * 16384.0).round().clamp(-32768.0, 32767.0) as i16).collect();

    let codec_max = f64::from(ucode::linear(target.law, 127));
    let margin = 10f64.powf(target.margin_db / 20.0);
    let mut candidates: Vec<(Cpd, f64)> = Vec::new();
    let mut enough_points = false;
    for step in 0..LARGEST_STEPS {
        let largest = codec_max * LARGEST_RATIO.powi(step);
        let mut spacing = SIGMAS * (channel.noise + e.residual * largest * largest / 3.0).sqrt() * margin;
        for _ in 0..3 {
            let points = constellation(target.law, largest, spacing, target.echo.then_some(SIGMAS * margin));
            let Ok(_) = rate(points.len(), 1.0) else { break };
            enough_points = true;
            // Largest moduli first; smaller ones put more points in each
            // class, which is what lets the precoder pull a large feedback in.
            let tried = [1.0, 0.75, 0.5].into_iter().find_map(|share| {
                let (moduli, drn) = rate(points.len(), share).ok()?;
                let cpd = Cpd {
                    drn,
                    trellis: UpTrellis::States16,
                    extend_e2u: false,
                    acknowledge: false,
                    gain4: gain4 as u16,
                    moduli: Some(moduli),
                    filters: Some(Filters { z1: vec![], p1: p1.clone(), z2: z2.clone(), p2: vec![] }),
                    constellations: Some(UpConstellations { intervals: [0; 6], sets: vec![points.clone()] }),
                };
                let r = rehearse(&cpd, &channel.h, e.delay, QUICK_FRAMES)?;
                (r.crest <= MAX_CREST && r.power <= MAX_POWER * target.power).then_some((cpd, r))
            });
            let Some((cpd, r)) = tried else { break };
            let needed = SIGMAS * (channel.noise + r.isi).sqrt() * margin;
            if needed > spacing * 1.05 {
                spacing = needed * 1.05;
                continue;
            }
            candidates.push((cpd, spacing));
            break;
        }
    }
    // The search's short rehearsals rank; a full one confirms, best first.
    candidates.sort_by(|(a, sa), (b, sb)| (b.drn, *sb).partial_cmp(&(a.drn, *sa)).unwrap_or(std::cmp::Ordering::Equal));
    for (cpd, spacing) in candidates {
        if !target.limits.admits(&cpd) {
            continue;
        }
        let confirmed = rehearse(&cpd, &channel.h, e.delay, FULL_FRAMES)
            .is_some_and(|r| r.crest <= MAX_CREST && r.power <= MAX_POWER * target.power);
        if confirmed {
            return Ok((Design { cpd, delay: e.delay }, spacing));
        }
    }
    Err(if enough_points { DesignError::NoFit } else { DesignError::TooNoisy })
}

/// The largest point's grid: the codec's top, then down by this ratio.
const LARGEST_RATIO: f64 = 0.85;
const LARGEST_STEPS: i32 = 20;
/// Mean-square transmit power allowed over the target, after rehearsal.
const MAX_POWER: f64 = 1.2;
/// Rehearsal lengths in data frames: the search's, and the winner's check.
const QUICK_FRAMES: usize = 250;
const FULL_FRAMES: usize = 1000;

/// Positive points up to `largest`, 128 at most.
///
/// `spread` None, a line without our echo: the codec's own levels, the
/// first at least half `spacing` from zero and each at least `spacing` plus
/// the codec's local step past the one before. The step is there because
/// the codec rounds before anything decides: a sample that misses its point
/// by more than half a step arrives a whole step away, and with points only
/// a step or two apart that whole step is most of the way to the neighbour.
/// With the step added, the distance to the decision boundary in what
/// arrives is spacing/2 again.
///
/// `spread` Some(k), our echo on the line: the codec rounds point plus echo,
/// so its rounding is noise spread over the cell, a twelfth of the local
/// step squared -- small at the bottom, and past everything else at the top
/// (measured on the ATA line, 2026-11-12: 17 rms at points under 479, 25
/// from 991, 33 and more from 2015, against a spacing made for 17.6
/// throughout). `spacing` is k standard deviations of everything else; each
/// gap is k standard deviations of that and the rounding where the gap
/// ends. The points need not be codec levels: nothing arrives on one.
fn constellation(law: Law, largest: f64, spacing: f64, spread: Option<f64>) -> Vec<u16> {
    let mut points: Vec<u16> = Vec::new();
    let Some(k) = spread else {
        for u in 1..=127u8 {
            let level = ucode::linear(law, u);
            let step = f64::from(level - ucode::linear(law, u - 1));
            if level <= 0 || f64::from(level) > largest || points.len() == 128 {
                continue;
            }
            let far_enough = match points.last() {
                None => 2.0 * f64::from(level) >= spacing + step,
                Some(&prev) => f64::from(level) - f64::from(prev) >= spacing + step,
            };
            if far_enough {
                points.push(level as u16);
            }
        }
        return points;
    };
    // The gap that ends at a point depends on the step there, which depends
    // on the gap: a few rounds settle it, the step only ever growing.
    let gap_from = |from: f64, share: f64| {
        let mut gap = spacing;
        for _ in 0..4 {
            let step = codec_step(law, from + share * gap);
            gap = (spacing * spacing + k * k * step * step / 12.0).sqrt();
        }
        gap
    };
    let mut at = (gap_from(0.0, 0.5) / 2.0).ceil();
    while at <= largest && points.len() < 128 {
        points.push(at as u16);
        at = (at + gap_from(at, 1.0)).ceil();
    }
    points
}

/// The codec's step where a sample of this size is rounded.
fn codec_step(law: Law, size: f64) -> f64 {
    let u = (1..=127u8).find(|&u| f64::from(ucode::linear(law, u)) >= size).unwrap_or(127);
    f64::from(ucode::linear(law, u) - ucode::linear(law, u - 1))
}

/// Moduli for `n` points, `share` of the most they allow (n at positions 0
/// to 2 of each four, n/2 at position 3), and the highest drn they carry.
fn rate(n: usize, share: f64) -> Result<([u8; 12], u8), DesignError> {
    if n < 2 {
        return Err(DesignError::TooNoisy);
    }
    let most = |m: usize| ((m as f64 * share).floor() as usize).clamp(1, 255) as u8;
    let moduli: [u8; 12] = std::array::from_fn(|i| if i % 4 == 3 { most(n / 2) } else { most(n) });
    let bits = moduli.iter().map(|&m| f64::from(m).log2()).sum::<f64>().floor() as u32;
    let drn = (1..=19u8).rev().find(|&d| 2 * (u32::from(d) + 17) <= bits).ok_or(DesignError::TooNoisy)?;
    Ok((moduli, drn))
}

/// Peak over RMS at the transmitter past which the precoder is taken to have
/// run away: x roughly uniform gives under 2, a few large feedback sums 3
/// or 4, and a precoder that cannot hold its feedback goes far past.
const MAX_CREST: f64 = 6.0;

/// What a rehearsal found.
#[derive(Debug, Clone, Copy)]
struct Rehearsal {
    /// Mean square at the transmitter.
    power: f64,
    /// Peak magnitude at the transmitter over its RMS.
    crest: f64,
    /// Mean square of codec input less the point sent.
    isi: f64,
}

/// The designed transmitter over `h` without noise, 1000 frames of
/// pseudo-random data, the first 50 left out while the filters fill. None if
/// the CPd is not one milestone 2 takes or the output is not finite.
fn rehearse(cpd: &Cpd, h: &[f64], delay: usize, frames: usize) -> Option<Rehearsal> {
    const WARM: usize = 50 * 12;
    let params = UpParams::from_cpd(cpd).ok()?;
    let mut tx = Transmitter::new(params.clone());
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    let mut samples = Vec::with_capacity(frames * 12);
    let mut points = Vec::with_capacity(frames * 12);
    for _ in 0..frames {
        let bits: Vec<bool> = (0..params.k)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                seed & 1 == 1
            })
            .collect();
        let sent = tx.frame(&bits)?;
        for i in 0..12 {
            samples.push(sent.samples[i]);
            points.push(f64::from(params.constellation(i).a(sent.etas[i])));
        }
    }
    if !samples.iter().all(|v| v.is_finite()) {
        return None;
    }
    let tail = &samples[WARM..];
    let power = tail.iter().map(|v| v * v).sum::<f64>() / tail.len() as f64;
    let peak = tail.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let crest = if power > 0.0 { peak / power.sqrt() } else { f64::INFINITY };
    let mut error = 0.0;
    let mut count = 0usize;
    for n in WARM..points.len().saturating_sub(delay) {
        let m = n + delay;
        let y: f64 = h.iter().enumerate().filter(|&(k, _)| k <= m).map(|(k, c)| c * samples[m - k]).sum();
        error += (y - points[n]).powi(2);
        count += 1;
    }
    (count > 0).then(|| Rehearsal { power, crest, isi: error / count as f64 })
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Rng(u64);
    impl Rng {
        fn uniform(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        fn gaussian(&mut self) -> f64 {
            let (u, v) = (self.uniform().max(1e-300), self.uniform());
            (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
        }
    }

    pub(super) fn convolve(h: &[f64], x: &[f64]) -> Vec<f64> {
        (0..x.len()).map(|n| h.iter().enumerate().filter(|(k, _)| *k <= n).map(|(k, c)| c * x[n - k]).sum()).collect()
    }

    #[test]
    fn trn1u_starts_with_five_negative_signs() {
        // 8.5.7: GPA (1 + x^-5 + x^-23: the answerer's taps in spandsp's
        // V.34, and the polynomial live V.90 upstream data descrambles with) from a zero state with ones in gives 1 until
        // the tap at 5 fills; "a scrambler output of 1 represents a negative
        // voltage".
        let s = trn1u_signs(6);
        assert!(s[..5].iter().all(|&v| v == -1.0), "{:?}", &s[..5]);
        // The 6th: 1 xor out[0] (1) xor 0 = 0, positive.
        assert_eq!(s[5], 1.0);
    }

    #[test]
    fn trn1u_is_balanced_and_white() {
        let s = trn1u_signs(4800);
        assert_eq!(s.len(), 4800);
        assert!(s.iter().all(|&v| v == 1.0 || v == -1.0));
        let mean = s.iter().sum::<f64>() / 4800.0;
        assert!(mean.abs() < 0.05, "mean {mean}");
        for lag in 1..=8 {
            let r = (lag..4800).map(|n| s[n] * s[n - lag]).sum::<f64>() / (4800 - lag) as f64;
            assert!(r.abs() < 0.06, "lag {lag}: {r}");
        }
    }

    #[test]
    fn estimate_recovers_a_known_channel() {
        let h = [300.0, 1200.0, -400.0, 100.0];
        let s = trn1u_signs(4800);
        let mut rng = Rng(7);
        let r: Vec<f64> = convolve(&h, &s).into_iter().map(|y| y + 20.0 * rng.gaussian()).collect();
        let c = estimate(&r, &s, 8);
        assert_eq!(c.h.len(), 8);
        for (k, want) in h.iter().chain([0.0; 4].iter()).enumerate() {
            assert!((c.h[k] - want).abs() < 15.0, "tap {k}: {} against {want}", c.h[k]);
        }
        assert!((c.noise - 400.0).abs() < 80.0, "noise {}", c.noise);
    }

    #[test]
    fn an_echo_is_undone_by_feedback_alone() {
        let e = equalise(&[1000.0, 500.0], 8, 8);
        assert!(e.residual < 1e-6, "residual {}", e.residual);
        assert_eq!(e.delay, 0, "{e:?}");
        assert!((e.p1[0] + 0.5).abs() < 1e-3, "p1 {:?}", e.p1);
    }

    #[test]
    fn a_delayed_peak_is_found() {
        let e = equalise(&[100.0, 400.0, 1000.0, 300.0], 8, 8);
        assert!(e.delay >= 1, "delay {}", e.delay);
        assert!(e.residual < 0.01, "residual {}", e.residual);
        assert!(e.p1.iter().all(|p| p.abs() < 2.0), "p1 {:?}", e.p1);
    }

    fn target(law: Law) -> Target {
        Target { law, power: 1.0, margin_db: 0.0, limits: Limits { sections: 0, ltot: 192, lmax: 128 }, echo: false }
    }

    fn request() -> PcmUpstreamRequest {
        PcmUpstreamRequest {
            filter_sections: 0,
            total_coefficient_limit: 192,
            section_coefficient_limit: 128,
            md_units: 0,
            uinfo: 80,
        }
    }

    #[test]
    fn an_empty_channel_gives_no_equaliser_not_a_panic() {
        assert!(equalise_with(&[], 0, 0, 1e-3).residual.is_infinite());
        assert!(equalise_with(&[], 4, 2, 1e-3).residual.is_infinite());
        assert!(equalise_with(&[1.0], 0, 2, 1e-3).residual.is_infinite());
    }

    #[test]
    fn the_limits_admit_z1_and_p2_only_where_info1a_s_sections_allow() {
        let cpd = |z1: usize, p2: usize| Cpd {
            filters: Some(Filters { z1: vec![1; z1], p1: vec![1; 4], z2: vec![1; 4], p2: vec![1; p2] }),
            ..design(&Channel { h: vec![2000.0], noise: 100.0 }, &target(Law::Mu)).unwrap().cpd
        };
        let limits = |sections| Limits { sections, ltot: 192, lmax: 128 };
        assert!(limits(0).admits(&cpd(0, 0)));
        assert!(!limits(0).admits(&cpd(2, 0)) && limits(1).admits(&cpd(2, 0)));
        assert!(!limits(1).admits(&cpd(0, 2)) && limits(2).admits(&cpd(0, 2)));
        assert!(!limits(3).admits(&cpd(0, 190)), "past Ltot");
        assert!(!limits(3).admits(&Cpd { filters: Some(Filters { z1: vec![], p1: vec![1; 129], z2: vec![], p2: vec![] }), ..cpd(0, 0) }), "past Lmax");
    }

    #[test]
    fn every_design_keeps_to_its_limits() {
        for sections in 0..=3 {
            let t = Target { limits: Limits { sections, ltot: 192, lmax: 128 }, ..target(Law::Mu) };
            let d = design(&Channel { h: vec![300.0, 1200.0, -400.0, 100.0], noise: 100.0 }, &t).unwrap();
            assert!(t.limits.admits(&d.cpd), "sections {sections}");
        }
    }

    #[test]
    fn the_cpd_is_one_milestone_two_accepts() {
        let d = design(&Channel { h: vec![2000.0], noise: 100.0 }, &target(Law::Mu)).unwrap();
        super::super::params::UpParams::from_cpd(&d.cpd).unwrap();
        assert!(d.cpd.fits(&request()));
        assert!(d.cpd.drn >= 12, "drn {}", d.cpd.drn);
        assert_eq!(d.delay, 0);
    }

    #[test]
    fn more_noise_means_a_lower_rate() {
        let quiet = design(&Channel { h: vec![2000.0], noise: 100.0 }, &target(Law::Mu)).unwrap();
        let loud = design(&Channel { h: vec![2000.0], noise: 900.0 }, &target(Law::Mu)).unwrap();
        assert!(loud.cpd.drn < quiet.cpd.drn, "{} against {}", loud.cpd.drn, quiet.cpd.drn);
    }

    #[test]
    fn too_much_noise_is_an_error() {
        let r = design(&Channel { h: vec![2000.0], noise: 3000.0 * 3000.0 }, &target(Law::Mu));
        assert_eq!(r.unwrap_err(), DesignError::TooNoisy);
    }

    #[test]
    fn the_codec_s_rounding_is_not_counted_as_noise() {
        let h = [300.0, 1200.0, -400.0, 100.0];
        let s = trn1u_signs(4800);
        let mut rng = Rng(3);
        let heard: Vec<f64> = convolve(&h, &s)
            .into_iter()
            .map(|y| {
                let y = y + 8.0 * rng.gaussian();
                let (u, neg) = ucode::nearest(Law::Mu, y.round() as i32);
                let v = f64::from(ucode::linear(Law::Mu, u));
                if neg { -v } else { v }
            })
            .collect();
        let raw = estimate(&heard, &s, 4);
        let clean = analogue(&raw, &heard, &s, Law::Mu);
        assert!(raw.noise > 2.0 * 64.0, "raw {}", raw.noise);
        eprintln!("raw {} clean {}", raw.noise, clean.noise);
        // Never under the truth by much (that would crowd the points), and
        // well under the raw figure.
        assert!(clean.noise > 0.8 * 64.0 && clean.noise < raw.noise / 2.0, "clean {} against 64, raw {}", clean.noise, raw.noise);
    }

    // ---- the sample path's split work against the one-shot originals ----

    /// [`estimate`] as it was before its normal equations were summed down
    /// the diagonals.
    fn estimate_reference(received: &[f64], signs: &[f64], taps: usize) -> Channel {
        let len = received.len().min(signs.len());
        let rows = (taps.saturating_sub(1)..len).collect::<Vec<_>>();
        let mut ata = vec![vec![0.0; taps]; taps];
        let mut atb = vec![0.0; taps];
        for &n in &rows {
            for i in 0..taps {
                atb[i] += signs[n - i] * received[n];
                for j in 0..taps {
                    ata[i][j] += signs[n - i] * signs[n - j];
                }
            }
        }
        let h = solve(ata, atb).unwrap_or_else(|| vec![0.0; taps]);
        let residual: f64 = rows
            .iter()
            .map(|&n| {
                let y: f64 = (0..taps).map(|k| h[k] * signs[n - k]).sum();
                (received[n] - y).powi(2)
            })
            .sum();
        let noise = if rows.len() > taps { residual / (rows.len() - taps) as f64 } else { 0.0 };
        Channel { h, noise }
    }

    fn equalise_with_reference(h: &[f64], lf: usize, lp: usize, ratio: f64) -> Equaliser {
        let none = || Equaliser { f: vec![0.0; lf], p1: vec![0.0; lp], delay: 0, residual: f64::INFINITY };
        if h.is_empty() || lf == 0 {
            return none();
        }
        let rows = h.len() + lf - 1;
        // Row n of the convolution matrix: (h*f)[n] = sum_j h[n-j] f[j].
        let a: Vec<Vec<f64>> = (0..rows)
            .map(|n| (0..lf).map(|j| if n >= j && n - j < h.len() { h[n - j] } else { 0.0 }).collect())
            .collect();
        let outside = |delay: usize, n: usize| n < delay || n > delay + lp;
        let add = |r: &mut Vec<Vec<f64>>, row: &[f64], sign: f64| {
            for i in 0..lf {
                if row[i] == 0.0 {
                    continue;
                }
                for j in 0..lf {
                    r[i][j] += sign * row[i] * row[j];
                }
            }
        };
        let mu = ratio * h.iter().map(|v| v * v).sum::<f64>();
        // R for one delay, from nothing: mu I plus the rows outside the window.
        let fresh = |delay: usize| {
            let mut r = vec![vec![0.0; lf]; lf];
            for (i, ri) in r.iter_mut().enumerate() {
                ri[i] = mu;
            }
            for n in (0..rows).filter(|&n| outside(delay, n)) {
                add(&mut r, &a[n], 1.0);
            }
            r
        };
        let mut best: Option<(bool, f64, Equaliser)> = None;
        let mut r = fresh(0);
        for delay in 0..lf.min(rows) {
            if delay > 0 {
                // Slide the window by one: row delay-1 leaves it, row
                // delay+lp enters it. Rebuilt every few delays against drift.
                if delay % 8 == 0 {
                    r = fresh(delay);
                } else {
                    add(&mut r, &a[delay - 1], 1.0);
                    if delay + lp < rows {
                        add(&mut r, &a[delay + lp], -1.0);
                    }
                }
            }
            let g = a[delay].clone();
            let Some(rg) = solve(r.clone(), g.clone()) else { continue };
            let norm: f64 = g.iter().zip(&rg).map(|(a, b)| a * b).sum();
            if norm.abs() < 1e-300 {
                continue;
            }
            let f: Vec<f64> = rg.iter().map(|v| v / norm).collect();
            let mut c = vec![0.0; rows];
            for (k, hk) in h.iter().enumerate() {
                for (j, fj) in f.iter().enumerate() {
                    c[k + j] += hk * fj;
                }
            }
            let p1: Vec<f64> = (1..=lp).map(|k| -c.get(delay + k).copied().unwrap_or(0.0)).collect();
            let residual: f64 = c.iter().enumerate().filter(|&(n, _)| outside(delay, n)).map(|(_, v)| v * v).sum();
            let fits = p1.iter().all(|p| p.abs() < 2.0 - 1.0 / 16384.0);
            let cost = residual + mu * f.iter().map(|v| v * v).sum::<f64>();
            let better = match &best {
                None => true,
                // A later delay is latency; it has to be clearly better.
                Some((best_fits, best_cost, _)) => {
                    (fits && !best_fits) || (fits == *best_fits && cost < best_cost * 0.99)
                }
            };
            if better {
                best = Some((fits, cost, Equaliser { f, p1, delay, residual }));
            }
        }
        best.map(|(_, _, e)| e).unwrap_or_else(none)
    }

    fn pseudo(n: usize, seed: u64) -> Vec<f64> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                s ^= s << 13;
                s ^= s >> 7;
                s ^= s << 17;
                ((s >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
            })
            .collect()
    }

    fn a_line() -> Vec<f64> {
        (0..30).map(|k| 1500.0 * (-(k as f64) / 6.0).exp() * if k % 2 == 0 { 1.0 } else { -0.5 }).collect()
    }

    #[test]
    fn the_diagonal_normal_equations_estimate_what_the_full_sums_did() {
        let h = a_line();
        // TRN1u's +-1 signs, and TRN2u-like levels that are not +-1.
        let ones: Vec<f64> = pseudo(4096, 7).iter().map(|v| v.signum()).collect();
        let levels: Vec<f64> = pseudo(4096, 9).iter().map(|v| (v * 4.0).round() / 2.0 + 0.25).collect();
        let noise = pseudo(4096, 11);
        for signs in [&ones, &levels] {
            let rx: Vec<f64> = (0..4096)
                .map(|n| (0..h.len()).filter(|&k| k <= n).map(|k| h[k] * signs[n - k]).sum::<f64>() + 40.0 * noise[n])
                .collect();
            for (len, taps) in [(4096, 80), (2040, 80), (300, 8), (100, 4), (80, 80), (79, 80)] {
                let fast = estimate(&rx[..len], &signs[..len], taps);
                let slow = estimate_reference(&rx[..len], &signs[..len], taps);
                assert_eq!(fast.h.len(), slow.h.len());
                for (a, b) in fast.h.iter().zip(&slow.h) {
                    assert!((a - b).abs() <= 1e-6 * (1.0 + b.abs()), "{len}x{taps}: tap {a} against {b}");
                }
                assert!((fast.noise - slow.noise).abs() <= 1e-6 * (1.0 + slow.noise), "{len}x{taps}: noise {} against {}", fast.noise, slow.noise);
            }
        }
    }

    #[test]
    fn the_equaliser_spread_over_steps_is_the_one_shot_one() {
        let est = estimate(
            &{
                let s: Vec<f64> = pseudo(2040, 3).iter().map(|v| v.signum()).collect();
                let h = a_line();
                (0..2040).map(|n| (0..h.len()).filter(|&k| k <= n).map(|k| h[k] * s[n - k]).sum::<f64>()).collect::<Vec<_>>()
            },
            &pseudo(2040, 3).iter().map(|v| v.signum()).collect::<Vec<_>>(),
            80,
        );
        for (h, lf, lp) in [(est.h.clone(), 64, 32), (est.h.clone(), 64, 0), (vec![100.0, 400.0, 1000.0, 300.0], 8, 8), (vec![1.0], 4, 2), (vec![], 4, 2), (vec![1.0], 0, 2)] {
            for ratio in [1e-3, 1e-4, 1e-5, 1e-6] {
                let want = equalise_with_reference(&h, lf, lp, ratio);
                assert_eq!(equalise_with(&h, lf, lp, ratio), want, "one shot, {lf}/{lp} at {ratio}");
                let mut job = EqualiseJob::new(&h, lf, lp, ratio);
                let mut steps = 0;
                let got = loop {
                    steps += 1;
                    if let Some(e) = job.step(1) {
                        break e;
                    }
                    assert!(steps <= lf + 2, "never finishes");
                };
                assert_eq!(got, want, "one delay a step, {lf}/{lp} at {ratio}");
                assert_eq!(job.step(1), Some(got), "a finished job stays finished");
            }
        }
    }

}

#[cfg(test)]
mod echo_line {
    use super::*;
    use crate::v90::ucode::Law;

    /// The local ATA line of 2026-11-10: a telephone band whose main tap is
    /// 482, and 16 rms left after echo cancelling (12 with this end silent).
    fn line() -> Vec<f64> {
        let fs = 8000.0;
        let sinc = |x: f64| if x == 0.0 { 1.0 } else { (std::f64::consts::PI * x).sin() / (std::f64::consts::PI * x) };
        let band: Vec<f64> = (0..31)
            .map(|n| {
                let m = n as f64 - 15.0;
                let w = 0.54 - 0.46 * (std::f64::consts::TAU * n as f64 / 30.0).cos();
                (2.0 * 3400.0 / fs * sinc(2.0 * 3400.0 * m / fs) - 2.0 * 300.0 / fs * sinc(2.0 * 300.0 * m / fs)) * w
            })
            .collect();
        let peak = band.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        band.iter().map(|v| v * 482.0 / peak).collect()
    }

    fn target(echo: bool) -> Target {
        Target { law: Law::Mu, power: 1.0, margin_db: 0.0, limits: Limits { sections: 0, ltot: 192, lmax: 128 }, echo }
    }

    #[test]
    fn with_our_echo_the_codec_s_rounding_comes_out_of_the_measured_noise() {
        let h = [300.0, 1200.0, -400.0, 100.0];
        let s = trn1u_signs(9600);
        let mut seed = 5u64;
        let mut uniform = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let heard: Vec<f64> = super::tests::convolve(&h, &s)
            .into_iter()
            .map(|y| {
                // 13 rms of noise, an echo of about a hundred, the codec,
                // and the canceller taking the echo out again: the ATA line.
                let noise = 13.0 * (0..12).map(|_| uniform() - 0.5).sum::<f64>();
                let echo = 350.0 * (uniform() - 0.5);
                let (u, neg) = ucode::nearest(Law::Mu, (y + noise + echo).round() as i32);
                let v = f64::from(ucode::linear(Law::Mu, u));
                (if neg { -v } else { v }) - echo
            })
            .collect();
        let raw = estimate(&heard, &s, 4);
        let clean = unrounded(&raw, &heard, Law::Mu);
        assert!(raw.noise > 2.0 * 169.0, "raw {}", raw.noise);
        assert!(clean.noise > 0.7 * 169.0 && clean.noise < 1.3 * 169.0, "clean {} against 169, raw {}", clean.noise, raw.noise);
    }

    #[test]
    fn with_our_echo_the_points_open_out_as_the_codec_s_steps_grow() {
        let points = constellation(Law::Mu, 4000.0, 100.0, Some(6.0));
        assert!(points.len() > 12, "{points:?}");
        assert!(f64::from(points[1] - points[0]) < 106.0, "{points:?}");
        for w in points.windows(2) {
            let step = codec_step(Law::Mu, f64::from(w[1]));
            let needed = (100.0f64.powi(2) + 36.0 * step * step / 12.0).sqrt();
            assert!(f64::from(w[1] - w[0]) >= needed - 1.0, "{} to {} wants {needed}: {points:?}", w[0], w[1]);
        }
        // Mu-law's step is 128 from 2015: 6 sigma of 100/6 and 128/sqrt(12).
        let top: Vec<u16> = points.iter().copied().filter(|&p| p > 2200).collect();
        assert!(top.windows(2).all(|w| w[1] - w[0] >= 243), "{top:?}");
    }

    #[test]
    fn a_weak_line_with_our_echo_on_it_still_gets_the_lowest_rates() {
        let d = design(&Channel { h: line(), noise: 16.0 * 16.0 }, &target(true)).expect("a design");
        assert!((1..=4).contains(&d.cpd.drn), "drn {}", d.cpd.drn);
        // Without echo the same figure is real noise on top of a codec that
        // rounds the points themselves: no room.
        assert_eq!(design(&Channel { h: line(), noise: 16.0 * 16.0 }, &target(false)).unwrap_err(), DesignError::TooNoisy);
    }
}
