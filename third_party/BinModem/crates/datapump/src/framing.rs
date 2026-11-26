//! Start-stop (asynchronous) character framing.

/// Recovers 8N1-style characters from a sliced baseband level.
///
/// Async framing does not need a continuously-tracked bit clock: the line idles
/// at mark and every character re-synchronises on its own start bit, exactly as
/// a UART does. What it does need is rejection of glitches that look like start
/// bits, so a candidate edge is confirmed at the half-bit point before the
/// character is accepted, and the stop bit must come back as mark.
#[derive(Debug, Clone)]
pub struct AsyncFramer {
    sps: f64,
    data_bits: u32,
    state: State,
    since_edge: f64,
    next_bit: u32,
    value: u32,
    prev: f64,
    /// Characters whose stop bit was not mark.
    pub framing_errors: u64,
    /// Level of the most recently sampled data bit, for display. Set at each
    /// bit centre and cleared when read, so a scope sees one entry per bit.
    sampled: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    ConfirmStart,
    Data,
    Stop,
}

impl AsyncFramer {
    pub fn new(baud: f64, fs: f64, data_bits: u32) -> Self {
        assert!((5..=8).contains(&data_bits), "data_bits must be 5..=8");
        Self {
            sps: fs / baud,
            data_bits,
            state: State::Idle,
            since_edge: 0.0,
            next_bit: 0,
            value: 0,
            prev: 1.0,
            framing_errors: 0,
            sampled: None,
        }
    }

    /// Feed one sample of sliced level (`> 0` is mark) plus the carrier state.
    /// Returns a character once a full frame has been validated.
    pub fn feed(&mut self, level: f64, carrier: bool) -> Option<u8> {
        if !carrier {
            self.state = State::Idle;
            self.prev = level;
            return None;
        }

        let mut out = None;
        match self.state {
            State::Idle => {
                // A start bit is a mark-to-space transition on an idle line.
                if self.prev > 0.0 && level <= 0.0 {
                    self.state = State::ConfirmStart;
                    self.since_edge = 0.0;
                }
            }
            State::ConfirmStart => {
                self.since_edge += 1.0;
                if self.since_edge >= self.sps * 0.5 {
                    if level > 0.0 {
                        self.state = State::Idle; // glitch, not a start bit
                    } else {
                        self.state = State::Data;
                        self.next_bit = 0;
                        self.value = 0;
                    }
                }
            }
            State::Data => {
                self.since_edge += 1.0;
                // Bit n is sampled at its centre: 1.5 bit times past the edge,
                // then one bit time per bit after that.
                if self.since_edge >= self.sps * (1.5 + self.next_bit as f64) {
                    self.sampled = Some(level);
                    self.value |= u32::from(level > 0.0) << self.next_bit; // LSB first
                    self.next_bit += 1;
                    if self.next_bit >= self.data_bits {
                        self.state = State::Stop;
                    }
                }
            }
            State::Stop => {
                self.since_edge += 1.0;
                if self.since_edge >= self.sps * (1.5 + self.data_bits as f64) {
                    if level > 0.0 {
                        out = Some(self.value as u8);
                    } else {
                        self.framing_errors += 1;
                    }
                    self.state = State::Idle;
                }
            }
        }
        self.prev = level;
        out
    }

    /// Level of the data bit sampled on this call, if one was. Cleared by
    /// reading, so a display receives exactly one value per recovered bit.
    pub fn take_sampled(&mut self) -> Option<f64> {
        self.sampled.take()
    }

    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.prev = 1.0;
        self.sampled = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Render bytes as an idealised 8N1 level waveform.
    fn wave(bytes: &[u8], sps: usize) -> Vec<f64> {
        let mut v = vec![1.0; sps * 4]; // idle mark
        for &b in bytes {
            v.extend(std::iter::repeat_n(-1.0, sps)); // start
            for i in 0..8 {
                let bit = (b >> i) & 1;
                v.extend(std::iter::repeat_n(if bit == 1 { 1.0 } else { -1.0 }, sps));
            }
            v.extend(std::iter::repeat_n(1.0, sps * 2)); // stop + idle
        }
        v
    }

    #[test]
    fn round_trips_ascii() {
        let msg = b"login:CACTUS\r\n";
        let sps = 53usize; // 16000 / 300, truncated as a real receiver would
        let mut f = AsyncFramer::new(300.0, 300.0 * sps as f64, 8);
        let got: Vec<u8> = wave(msg, sps)
            .into_iter()
            .filter_map(|l| f.feed(l, true))
            .collect();
        assert_eq!(got, msg, "got {:?}", String::from_utf8_lossy(&got));
        assert_eq!(f.framing_errors, 0);
    }

    #[test]
    fn rejects_a_half_bit_glitch() {
        let sps = 53usize;
        let mut f = AsyncFramer::new(300.0, 300.0 * sps as f64, 8);
        let mut v = vec![1.0; sps * 4];
        v.extend(std::iter::repeat_n(-1.0, sps / 8)); // far too short to be a start bit
        v.extend(std::iter::repeat_n(1.0, sps * 4));
        let got: Vec<u8> = v.into_iter().filter_map(|l| f.feed(l, true)).collect();
        assert!(got.is_empty(), "glitch produced {got:?}");
    }

    #[test]
    fn loss_of_carrier_aborts_a_partial_character() {
        let sps = 53usize;
        let mut f = AsyncFramer::new(300.0, 300.0 * sps as f64, 8);
        let v = wave(b"A", sps);
        let half = v.len() / 2;
        for &l in &v[..half] {
            f.feed(l, true);
        }
        assert!(f.feed(0.0, false).is_none());
        for &l in &v[half..] {
            assert!(f.feed(l, true).is_none(), "resumed a torn character");
        }
    }
}

/// Start-stop character framing carried over a synchronous bit stream (V.14).
///
/// A modem's line is synchronous: bits go out at the symbol rate whether or
/// not anything wants to send one. A terminal is asynchronous: it sends
/// characters when it has them, each announced by a start bit and closed by a
/// stop bit, and says nothing in between. V.14 is the conversion, and without
/// it the far end has a stream of bits and no idea where one character ends
/// and the next begins.
///
/// The framing is what makes the boundary findable. Idle is mark, so the
/// falling edge into a start bit is unmistakable, and every character
/// re-synchronises on its own: a receiver that joins a call halfway through
/// needs to find one start bit and is then in step.
///
/// What is not done here is the part V.14 is really about. The two clocks are
/// never quite equal, so over a long transfer the asynchronous side delivers
/// slightly more or fewer characters than the synchronous side has room for,
/// and V.14 5.2 recovers the difference by deleting or inserting stop bits.
/// That matters between two independently clocked machines and not at all
/// between two ends of one program.
#[derive(Debug, Clone)]
pub struct AsyncBits {
    data_bits: u32,
    state: BitState,
    /// Bits of the character gathered so far, and how many.
    value: u32,
    have: u32,
    /// Characters whose stop bit was not mark.
    errors: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BitState {
    /// Waiting for a start bit, which is the only space in an idle line.
    Idle,
    Data,
    Stop,
}

impl AsyncBits {
    pub fn new(data_bits: u32) -> Self {
        Self {
            data_bits: data_bits.clamp(5, 8),
            state: BitState::Idle,
            value: 0,
            have: 0,
            errors: 0,
        }
    }

    /// Wrap one character: a start bit, the data least significant first, and
    /// a stop bit.
    pub fn encode(&self, byte: u8) -> Vec<bool> {
        let mut out = Vec::with_capacity(self.data_bits as usize + 2);
        out.push(false);
        for i in 0..self.data_bits {
            out.push(byte & (1 << i) != 0);
        }
        out.push(true);
        out
    }

    /// Feed one received bit, yielding a character when one completes.
    pub fn feed(&mut self, bit: bool) -> Option<u8> {
        match self.state {
            BitState::Idle => {
                if !bit {
                    self.state = BitState::Data;
                    self.value = 0;
                    self.have = 0;
                }
                None
            }
            BitState::Data => {
                if bit {
                    self.value |= 1 << self.have;
                }
                self.have += 1;
                if self.have == self.data_bits {
                    self.state = BitState::Stop;
                }
                None
            }
            BitState::Stop => {
                self.state = BitState::Idle;
                if bit {
                    Some(self.value as u8)
                } else {
                    // The stop bit was not mark, so the framing was wrong
                    // somewhere and the character cannot be trusted. Dropping
                    // it and hunting for the next start bit is what a UART
                    // does and recovers within a character or two.
                    self.errors += 1;
                    None
                }
            }
        }
    }

    /// Characters discarded because their stop bit was not mark.
    pub fn framing_errors(&self) -> u64 {
        self.errors
    }

    pub fn reset(&mut self) {
        self.state = BitState::Idle;
        self.value = 0;
        self.have = 0;
        // Including the count. It belongs to a call: carrying it into the next
        // one makes a fresh link look like it inherited somebody else's
        // trouble, and makes any rate worked out from it wrong at the start.
        self.errors = 0;
    }
}

#[cfg(test)]
mod async_bits_tests {
    use super::AsyncBits;

    #[test]
    fn characters_survive_the_round_trip() {
        let framer = AsyncBits::new(8);
        let mut back = AsyncBits::new(8);
        let mut out = Vec::new();
        for byte in b"Welcome to phl6-dial1" {
            for bit in framer.encode(*byte) {
                if let Some(c) = back.feed(bit) {
                    out.push(c);
                }
            }
        }
        assert_eq!(out, b"Welcome to phl6-dial1");
    }

    #[test]
    fn a_receiver_joining_halfway_finds_the_boundary() {
        // The reason for start bits. A synchronous line hands over a stream
        // with no marks in it, and where a character begins is not something
        // the receiver can be told.
        let framer = AsyncBits::new(8);
        let mut bits: Vec<bool> = Vec::new();
        // Idle first, which is what the receiver joins in the middle of.
        bits.extend(std::iter::repeat_n(true, 13));
        for byte in b"cactus" {
            bits.extend(framer.encode(*byte));
        }
        let mut back = AsyncBits::new(8);
        let out: Vec<u8> = bits.iter().filter_map(|&b| back.feed(b)).collect();
        assert_eq!(out, b"cactus");
    }

    #[test]
    fn idle_produces_no_characters() {
        let mut back = AsyncBits::new(8);
        for _ in 0..1000 {
            assert_eq!(back.feed(true), None);
        }
    }

    #[test]
    fn a_bad_stop_bit_costs_one_character_and_no_more() {
        // A UART recovers by hunting for the next start bit, and so does this:
        // one character is lost and the rest arrive.
        let framer = AsyncBits::new(8);
        let mut bits: Vec<bool> = Vec::new();
        bits.extend(framer.encode(b'a'));
        let mut broken = framer.encode(b'b');
        let last = broken.len() - 1;
        broken[last] = false;
        bits.extend(broken);
        bits.extend(std::iter::repeat_n(true, 4));
        bits.extend(framer.encode(b'c'));

        let mut back = AsyncBits::new(8);
        let out: Vec<u8> = bits.iter().filter_map(|&b| back.feed(b)).collect();
        assert_eq!(out, b"ac");
        assert_eq!(back.framing_errors(), 1);
    }
}

/// A bit clock for a synchronous stream carried on a sliced FSK level.
///
/// V.42 over Bell 103 sends HDLC frames on the FSK carrier, with no start
/// bits to re-synchronise on, so the bits have to be taken at a recovered
/// clock. Each level transition marks a bit boundary; the sampling instant is
/// pulled halfway towards half a bit after it. Pulling instead of jumping
/// keeps a noise spike from inserting a bit, and taking the correction modulo
/// one bit keeps it from ever sampling the same bit twice.
#[derive(Debug, Clone)]
pub struct BitClock {
    sps: f64,
    t: f64,
    next: f64,
    prev: bool,
}

impl BitClock {
    pub fn new(baud: f64, fs: f64) -> Self {
        let sps = fs / baud;
        Self { sps, t: 0.0, next: sps * 0.5, prev: true }
    }

    /// One sample of sliced level (`> 0` is mark); a bit when one is due.
    pub fn feed(&mut self, level: f64) -> Option<bool> {
        let bit = level > 0.0;
        self.t += 1.0;
        if bit != self.prev {
            let want = self.t + self.sps * 0.5;
            let mut d = (want - self.next) % self.sps;
            if d > self.sps * 0.5 { d -= self.sps; }
            if d < -self.sps * 0.5 { d += self.sps; }
            // Half a bit off, either way is as near. Going back so far that
            // the next instant is already past would sample at once and again
            // within this same bit; go forward instead.
            if self.next + 0.5 * d <= self.t { d += self.sps; }
            self.next += 0.5 * d;
            self.prev = bit;
        }
        if self.t >= self.next {
            self.next += self.sps;
            return Some(bit);
        }
        None
    }

    pub fn reset(&mut self) {
        self.t = 0.0;
        self.next = self.sps * 0.5;
        self.prev = true;
    }
}

#[cfg(test)]
mod bit_clock_tests {
    use super::BitClock;

    fn levels(bits: &[bool], sps: f64, noise_at: Option<usize>) -> Vec<f64> {
        let mut v = Vec::new();
        let mut t = 0.0f64;
        for &b in bits {
            t += sps;
            while (v.len() as f64) < t { v.push(if b { 1.0 } else { -1.0 }); }
        }
        if let Some(n) = noise_at { v[n] = -v[n]; }
        v
    }

    fn recover(bits: &[bool], sps: f64, noise_at: Option<usize>) -> Vec<bool> {
        let mut c = BitClock::new(300.0, 8000.0);
        levels(bits, sps, noise_at).iter().filter_map(|&l| c.feed(l)).collect()
    }

    fn pattern() -> Vec<bool> {
        // Flags, then data with long runs both ways, as HDLC carries.
        let mut b = Vec::new();
        for byte in [0x7eu8, 0x7e, 0x00, 0x00, 0xff, 0x3e, 0xa5, 0x7e, 0x13, 0x00, 0x7e] {
            for i in 0..8 { b.push(byte >> i & 1 != 0); }
        }
        b
    }

    #[test]
    fn a_synchronous_stream_comes_back_bit_for_bit() {
        let bits = pattern();
        assert_eq!(recover(&bits, 8000.0 / 300.0, None), bits);
    }

    #[test]
    fn a_far_clock_half_a_percent_fast_is_tracked() {
        let bits: Vec<bool> = pattern().repeat(20);
        assert_eq!(recover(&bits, 8000.0 / 301.5, None), bits);
    }

    /// The discriminator wobbles across zero for a few samples at a slow
    /// transition, worst at the first start bit after a long idle mark. A
    /// live Bell 103 call lost its first typed character to that: each wobble
    /// pulled the clock and the start bit was sampled twice.
    #[test]
    fn a_ragged_edge_does_not_sample_a_bit_twice() {
        let mut bits = vec![true; 300];
        bits.extend(pattern());
        let sps = 8000.0 / 300.0;
        let mut v = levels(&bits, sps, None);
        // Chatter at every transition: the new level, then the old, for a
        // few samples before settling.
        let edges: Vec<usize> = (1..v.len()).filter(|&i| (v[i] > 0.0) != (v[i - 1] > 0.0)).collect();
        for e in edges {
            for k in [1usize, 3] {
                if e + k < v.len() { v[e + k] = -v[e + k]; }
            }
        }
        let mut c = BitClock::new(300.0, 8000.0);
        let got: Vec<bool> = v.iter().filter_map(|&l| c.feed(l)).collect();
        assert_eq!(got, bits);
    }

    /// A start bit after a long idle mark, wherever it falls against the
    /// free-running clock. A live Bell 103 call lost its first typed
    /// character when the edge came half a bit from the sampling instant:
    /// the correction went backwards, sampled at once and again inside the
    /// same start bit.
    #[test]
    fn a_start_bit_after_idle_is_taken_once_wherever_it_falls() {
        use super::AsyncBits;
        let framer = AsyncBits::new(8);
        let sps = 8000.0 / 300.0;
        for idle in 2600..2680 {
            let mut v = vec![1.0; idle];
            let tail = levels(&[framer.encode(b'd'), framer.encode(b'i')].concat(), sps, None);
            v.extend(tail);
            v.extend(vec![1.0; 200]);
            let mut c = BitClock::new(300.0, 8000.0);
            let mut de = AsyncBits::new(8);
            let got: Vec<u8> = v.iter().filter_map(|&l| c.feed(l)).filter_map(|b| de.feed(b)).collect();
            assert_eq!(got, b"di".to_vec(), "idle {idle}");
        }
    }

    #[test]
    fn a_one_sample_spike_neither_adds_nor_drops_a_bit() {
        let bits = pattern();
        assert_eq!(recover(&bits, 8000.0 / 300.0, Some(200)), bits);
    }
}
