//! The precoder and prefilter of 6.4.2, and the choice of point within an
//! equivalence class.

use std::collections::VecDeque;

use super::params::{Constellation, TxFilters};

/// Newest first: `h[0]` is the value one symbol back.
fn dot(c: &[f64], h: &VecDeque<f64>) -> f64 {
    c.iter().zip(h).map(|(c, h)| c * h).sum()
}

fn push(h: &mut VecDeque<f64>, len: usize, v: f64) {
    if len == 0 {
        return;
    }
    h.push_front(v);
    h.truncate(len);
}

#[derive(Debug, Clone)]
pub struct Precoder {
    f: TxFilters,
    gain: f64,
    /// u(n - 1), u(n - 2), ...
    u: VecDeque<f64>,
    /// x(n - 1), ...
    x: VecDeque<f64>,
    /// v(n - 1), ...
    v: VecDeque<f64>,
}

impl Precoder {
    pub fn new(filters: TxFilters, gain: f64) -> Self {
        Self { f: filters, gain, u: VecDeque::new(), x: VecDeque::new(), v: VecDeque::new() }
    }

    /// x(n) - u(n): what the filters add from the past.
    pub fn offset(&self) -> f64 {
        dot(&self.f.z1, &self.u) + dot(&self.f.p1, &self.x)
    }

    /// The eta of `class` that makes |x(n)| least -- the precoder output's
    /// power, symbol by symbol, as Table 30's NOTE assumes. Ties go to the
    /// quieter point.
    pub fn choose(&self, constellation: &Constellation, class: impl Iterator<Item = i32>) -> i32 {
        let c = self.offset();
        class
            .min_by(|&a, &b| {
                let (pa, pb) = (f64::from(constellation.a(a)), f64::from(constellation.a(b)));
                (pa + c).abs().total_cmp(&(pb + c).abs()).then(pa.abs().total_cmp(&pb.abs()))
            })
            .unwrap_or(0)
    }

    /// Send u(n): returns G x v(n).
    pub fn send(&mut self, u: f64) -> f64 {
        let x = u + self.offset();
        // v(n) = sum z2(k) x(n - k), k from 0, + sum p2(k) v(n - k), k from 1.
        let v = self.f.z2.first().map_or(0.0, |z| z * x)
            + self.f.z2.iter().skip(1).zip(&self.x).map(|(z, x)| z * x).sum::<f64>()
            + dot(&self.f.p2, &self.v);
        push(&mut self.u, self.f.z1.len(), u);
        push(&mut self.x, self.f.p1.len().max(self.f.z2.len().saturating_sub(1)), x);
        push(&mut self.v, self.f.p2.len(), v);
        self.gain * v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v92::upstream::params::{Constellation, TxFilters};

    fn filters(z1: &[f64], p1: &[f64], z2: &[f64], p2: &[f64]) -> TxFilters {
        TxFilters { z1: z1.to_vec(), p1: p1.to_vec(), z2: z2.to_vec(), p2: p2.to_vec() }
    }

    #[test]
    fn precoder_by_hand() {
        // x(n) = u(n) + 0.5 u(n-1) + 0.5 x(n-1): 1, 1.0, 0.5.
        let mut p = Precoder::new(filters(&[0.5], &[0.5], &[1.0], &[]), 1.0);
        assert_eq!([p.send(1.0), p.send(0.0), p.send(0.0)], [1.0, 1.0, 0.5]);
    }

    #[test]
    fn prefilter_by_hand() {
        // v(n) = 0.5 x(n) + 0.25 x(n-1) - 0.5 v(n-1): 0.5, 0.25 - 0.25, 0; G 2.
        let mut p = Precoder::new(filters(&[], &[], &[0.5, 0.25], &[-0.5]), 2.0);
        assert_eq!([p.send(1.0), p.send(0.0), p.send(0.0)], [1.0, 0.0, 0.0]);
    }

    #[test]
    fn choice_minimises_the_precoder_output() {
        let c = Constellation::from_positive(&[10, 20, 30, 40, 50, 60, 70, 80]);
        let class = || [-7, -3, 1, 5].into_iter();
        let mut p = Precoder::new(filters(&[0.5], &[], &[1.0], &[]), 1.0);
        // Nothing sent yet: the quietest point, a(1) = 20.
        assert_eq!(p.choose(&c, class()), 1);
        p.send(-100.0);
        assert_eq!(p.offset(), -50.0);
        // |a + offset| least at a(5) = 60.
        assert_eq!(p.choose(&c, class()), 5);
    }
}
