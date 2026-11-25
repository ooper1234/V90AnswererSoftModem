//! The analogue modem's PCM upstream transmitter (Figure 2/V.92), for the
//! test modem: scrambler, modulus encoder, precoder with the trellis's
//! equivalence classes, prefilter and gain.

use super::modulus::ModulusEncoder;
use super::params::UpParams;
use super::precoder::Precoder;
use super::trellis::{class, inverse_map};
use crate::v32::{Mode, Scrambler};

/// One data frame on its way out.
#[derive(Debug, Clone, PartialEq)]
pub struct Sent {
    /// G x v(n) for intervals 0 to 11.
    pub samples: [f64; 12],
    /// The index eta of each u(n).
    pub etas: [i32; 12],
}

#[derive(Debug, Clone)]
pub struct Transmitter {
    params: UpParams,
    scrambler: Scrambler,
    modulus: ModulusEncoder,
    precoder: Precoder,
    /// The convolutional encoder's state; Y0 comes out of it.
    state: u8,
}

impl Transmitter {
    /// Scrambler from zero, d(-1) = 0 and trellis state 0 (to be confirmed
    /// against E2u/B1u when start-up is built).
    pub fn new(params: UpParams) -> Self {
        Self {
            scrambler: Scrambler::new(Mode::Answer),
            modulus: ModulusEncoder::new(false),
            precoder: Precoder::new(params.filters.clone(), params.gain),
            state: 0,
            params,
        }
    }

    /// One data frame of exactly K bits, b0 first; `None` for any other length.
    pub fn frame(&mut self, bits: &[bool]) -> Option<Sent> {
        if bits.len() != self.params.k as usize {
            return None;
        }
        let scrambled: Vec<bool> = bits.iter().map(|&b| self.scrambler.scramble(b)).collect();
        let ks = self.modulus.encode(&scrambled, &self.params.moduli);
        let mut sent = Sent { samples: [0.0; 12], etas: [0; 12] };
        for i in 0..12 {
            let position = i % 4;
            let constellation = self.params.constellation(i);
            let parity = position == 3 && {
                let start = i - 3;
                let sum: i32 = sent.etas[start..i].iter().sum();
                (sum.rem_euclid(2) == 1) != self.params.code.output(self.state)
            };
            let candidates = class(position, ks[i], self.params.moduli[i], constellation.n(), parity);
            let eta = self.precoder.choose(constellation, candidates.into_iter());
            sent.etas[i] = eta;
            sent.samples[i] = self.precoder.send(f64::from(constellation.a(eta)));
            if position == 3 {
                let etas = [sent.etas[i - 3], sent.etas[i - 2], sent.etas[i - 1], eta];
                // The 4T delays: one step of the encoder per trellis frame.
                self.state = self.params.code.next(self.state, inverse_map(etas));
            }
        }
        Some(sent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v92::upstream::fixture::{frames, params};
    use crate::v92::upstream::trellis::inverse_map;

    #[test]
    fn wrong_length_is_refused() {
        assert!(Transmitter::new(params()).frame(&[true; 47]).is_none());
    }

    #[test]
    fn the_sum_of_a_trellis_frame_has_y0_parity() {
        let p = params();
        let mut tx = Transmitter::new(p.clone());
        let mut state = 0u8;
        for f in frames(50, 48, 7) {
            let sent = tx.frame(&f).unwrap();
            for t in sent.etas.chunks(4) {
                let etas: [i32; 4] = t.try_into().unwrap();
                assert_eq!(etas.iter().sum::<i32>().rem_euclid(2) == 1, p.code.output(state));
                state = p.code.next(state, inverse_map(etas));
            }
        }
    }

    #[test]
    fn identity_filters_send_the_points() {
        let p = params();
        let mut tx = Transmitter::new(p.clone());
        for f in frames(20, 48, 3) {
            let sent = tx.frame(&f).unwrap();
            for i in 0..12 {
                assert_eq!(sent.samples[i], f64::from(p.constellation(i).a(sent.etas[i])));
            }
        }
    }
}
