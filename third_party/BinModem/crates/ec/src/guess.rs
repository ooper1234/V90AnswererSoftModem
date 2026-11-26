//! Following a V.42bis stream whose parameters nobody heard.
//!
//! A far end that compresses without an XID exchange this end could read has
//! settled on N2 and N7 somewhere this end cannot see: the defaults (6.4) if
//! it never heard this end's offer either, or the lower of its figures and
//! this end's if it did. A decoder built with the wrong pair does not
//! necessarily fail. One with more entries than the encoder never meets a
//! codeword it has not allocated, so it decodes everything into nonsense
//! and says nothing. One with fewer meets such a codeword after the
//! dictionaries part -- often soon, sometimes not for a while -- and fails.
//!
//! So several are run side by side, chosen so that each is at least the one
//! before in both figures; otherwise two can each be larger in one and both
//! go on silently. Whatever they agree on is delivered at once, which is
//! everything before the far end's switch into compressed mode, and after it,
//! while they disagree, output is held. A candidate that fails is dropped.
//! When one is left it is the answer. When several are still standing after
//! [`HOLD_BYTES`] of held output or [`HOLD_MS`] of holding, the smallest is
//! followed -- the larger ones are those that cannot fail -- and the others
//! go on decoding behind it, so that if it fails after all the next one up
//! takes over and the screen recovers instead of filling with codewords.

use crate::v42bis::{Decoder, Params};

/// Held output past which the smallest surviving candidate is followed.
pub const HOLD_BYTES: usize = 4096;
/// Holding time past which the same.
pub const HOLD_MS: u32 = 1000;

struct Candidate {
    params: Params,
    decoder: Decoder,
    /// Decoded and not yet delivered.
    held: Vec<u8>,
}

/// Several decoders for one stream, until one of them is the answer.
pub struct Guess {
    /// Smallest first.
    live: Vec<Candidate>,
    /// Whether the smallest is being followed rather than waited on.
    following: bool,
    held_ms: u32,
}

/// What became of the stream.
pub enum Verdict {
    /// Still more than one candidate; what can be delivered has been.
    Undecided,
    /// This one, with everything it decoded delivered.
    Settled(Params, Decoder),
    /// Every candidate failed on this input. Nothing of it was delivered.
    Failed,
}

impl std::fmt::Debug for Guess {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let live: Vec<Params> = self.live.iter().map(|c| c.params).collect();
        f.debug_struct("Guess")
            .field("live", &live)
            .field("following", &self.following)
            .field("held_ms", &self.held_ms)
            .finish()
    }
}

impl Guess {
    /// Candidates in any order, duplicates allowed. They should form a chain,
    /// each at least the one before in both N2 and N7.
    pub fn new(candidates: &[Params]) -> Self {
        let mut params: Vec<Params> = candidates.to_vec();
        params.sort_by_key(|p| (p.n2, p.n7));
        params.dedup();
        Self {
            live: params
                .into_iter()
                .map(|params| Candidate { params, decoder: Decoder::new(params), held: Vec::new() })
                .collect(),
            following: false,
            held_ms: 0,
        }
    }

    /// Whether a candidate's output is going to the terminal although others
    /// are still standing: compressed data is being decoded on a guess.
    pub fn following(&self) -> bool {
        self.following
    }

    /// 5.6's C-INIT for every candidate. What was held was built from a
    /// stream that is being thrown away, and goes with it.
    pub fn reinitialize(&mut self) {
        for c in &mut self.live {
            c.decoder = Decoder::new(c.params);
            c.held.clear();
        }
        self.held_ms = 0;
    }

    /// Decode `input` with every live candidate, delivering into `out`.
    pub fn decode(&mut self, input: &[u8], out: &mut Vec<u8>) -> Verdict {
        let mut survivors = Vec::with_capacity(self.live.len());
        for mut c in self.live.drain(..) {
            if c.decoder.decode(input, &mut c.held).is_ok() {
                survivors.push(c);
            }
        }
        self.live = survivors;
        self.release(out)
    }

    /// Time passing; held output is not held for ever.
    pub fn tick(&mut self, dt_ms: u32, out: &mut Vec<u8>) -> Verdict {
        if self.live.iter().any(|c| !c.held.is_empty()) {
            self.held_ms = self.held_ms.saturating_add(dt_ms);
        }
        self.release(out)
    }

    fn release(&mut self, out: &mut Vec<u8>) -> Verdict {
        if self.live.is_empty() {
            return Verdict::Failed;
        }
        if self.live.len() == 1 {
            let mut only = self.live.remove(0);
            out.append(&mut only.held);
            return Verdict::Settled(only.params, only.decoder);
        }
        if !self.following {
            let held = self.live.iter().map(|c| c.held.len()).max().unwrap_or(0);
            self.following = held >= HOLD_BYTES || self.held_ms >= HOLD_MS;
        }
        if self.following {
            // The smallest standing is the one followed; the rest decode
            // behind it, their output unwanted unless it fails.
            out.append(&mut self.live[0].held);
            for c in &mut self.live[1..] {
                c.held.clear();
            }
            self.held_ms = 0;
            return Verdict::Undecided;
        }
        // Deliver what every candidate agrees on.
        let first = &self.live[0].held;
        let agreed = self.live[1..].iter().fold(first.len(), |n, c| {
            n.min(first.iter().zip(&c.held).take_while(|(a, b)| a == b).count())
        });
        if agreed > 0 {
            out.extend_from_slice(&self.live[0].held[..agreed]);
            for c in &mut self.live {
                c.held.drain(..agreed);
            }
        }
        if self.live.iter().all(|c| c.held.is_empty()) {
            self.held_ms = 0;
        }
        Verdict::Undecided
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::v42bis::Encoder;

    fn compressed(params: Params, text: &[u8]) -> Vec<u8> {
        let mut e = Encoder::new(params);
        let mut out = Vec::new();
        e.encode(text, &mut out);
        e.flush(&mut out);
        out
    }

    /// What the stack offers: a chain, each at least the one before.
    fn candidates() -> Vec<Params> {
        [(512, 6), (1024, 32), (2048, 32), (2048, 250)].map(|(n2, n7)| Params { n2, n7 }).to_vec()
    }

    /// Text with long repeats, so that strings past 32 characters form.
    fn text() -> Vec<u8> {
        let mut t = Vec::new();
        for i in 0..400 {
            t.extend_from_slice(
                format!("Line {i}: the quick brown fox jumps over the lazy dog. Welcome to the board.\r\n")
                    .as_bytes(),
            );
        }
        t
    }

    /// Feed `stream` in 64-octet pieces with 20 ms between them.
    fn run(g: &mut Guess, stream: &[u8]) -> (Vec<u8>, Option<Params>) {
        let mut out = Vec::new();
        let mut settled: Option<(Params, Decoder)> = None;
        for chunk in stream.chunks(64) {
            if let Some((_, d)) = settled.as_mut() {
                d.decode(chunk, &mut out).expect("the settled decoder failed");
                continue;
            }
            if let Verdict::Settled(p, d) = g.decode(chunk, &mut out) {
                settled = Some((p, d));
                continue;
            }
            if let Verdict::Settled(p, d) = g.tick(20, &mut out) {
                settled = Some((p, d));
            }
        }
        (out, settled.map(|(p, _)| p))
    }

    #[test]
    fn every_candidate_is_followed_from_its_own_stream() {
        let text = text();
        for truth in candidates() {
            let mut g = Guess::new(&candidates());
            let (out, settled) = run(&mut g, &compressed(truth, &text));
            // Settled outright, or still following the right one with the
            // larger ones (which cannot fail) behind it.
            let following = settled.unwrap_or_else(|| g.live[0].params);
            assert_eq!(following, truth);
            // The end of the text is right, whatever a wrong guess on the way
            // cost the middle.
            let tail = &text[text.len() - 2000..];
            assert!(out.ends_with(tail), "{truth:?}: the terminal got something else at the end");
        }
    }

    #[test]
    fn the_defaults_come_through_without_a_wrong_octet() {
        let text = text();
        let mut g = Guess::new(&candidates());
        let (out, _) = run(&mut g, &compressed(Params { n2: 512, n7: 6 }, &text));
        assert!(out == text, "the terminal got something other than the far end sent");
    }

    #[test]
    fn plain_text_goes_straight_through() {
        let mut g = Guess::new(&candidates());
        let mut out = Vec::new();
        assert!(matches!(g.decode(b"login: ", &mut out), Verdict::Undecided));
        assert_eq!(out, b"login: ");
    }

    #[test]
    fn held_output_is_released_in_time() {
        let text = text();
        let stream = compressed(Params { n2: 512, n7: 6 }, &text[..300]);
        let mut g = Guess::new(&candidates());
        let mut out = Vec::new();
        let _ = g.decode(&stream, &mut out);
        let _ = g.tick(HOLD_MS, &mut out);
        assert_eq!(out, &text[..300]);
    }
}
