//! V.92 wire formats. These building blocks do not advertise a working
//! V.92 datapump: PCM upstream, quick-connect and hold procedures must be
//! implemented and verified before a live answerer offers them.
//! Authority: ITU-T V.92 (11/2000), Tables 15, 16, 32 and 33.

use crate::v34::info::{self, Info0, Info0d, Info1c};

pub mod quick_connect;
pub mod phase3;
pub mod phase4;
pub mod upstream;

/// Table 19: during shortened Phase 2 the caller supplies the QAM carrier
/// directly; no INFO1d probing report is exchanged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShortQamRequest {
    pub legacy: info::Info1aPcm,
    pub high_carrier: bool,
}

impl ShortQamRequest {
    pub fn to_bits(self)->Vec<bool> {
        let bits=self.legacy.to_bits();
        let mut payload=info::unframe(&bits,info::INFO1A_BITS).unwrap().to_vec();
        payload[21]=self.high_carrier;
        frame(&payload)
    }
    pub fn from_bits(bits:&[bool])->Option<Self> {
        let payload=info::unframe(bits,info::INFO1A_BITS)?;
        let legacy=info::Info1aPcm::from_bits(bits)?;
        if legacy.uinfo<=66 {return None;}
        Some(Self {legacy,high_carrier:payload[21]})
    }
}

/// V.8 Quick Connect request (Table 2), followed by normal CM.
/// This frame has repetition rather than an INFO CRC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuickConnectRequest {
    pub lapm: bool,
    /// QTS Ucode, or None for the on-hold cleardown indication.
    pub qts_ucode: Option<u8>,
}

impl QuickConnectRequest {
    const LEVELS: [u8; 15] = [61, 62, 63, 66, 67, 70, 71, 74, 75, 78, 79, 82, 83, 86, 87];

    pub fn to_bits(self) -> Option<Vec<bool>> {
        let code = match self.qts_ucode {
            Some(level) => Self::LEVELS.iter().position(|&v| v == level)?,
            None => 15,
        };
        let field = [false, false, false, self.lapm, code & 8 != 0,
            false, code & 4 != 0, code & 2 != 0, code & 1 != 0, true];
        let mut bits = Vec::with_capacity(60);
        for _ in 0..2 {
            bits.extend([true; 10]);
            bits.extend((0..10).map(|i| i % 2 != 0));
            bits.extend(field);
        }
        Some(bits)
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        if bits.len() != 60 || bits[..30] != bits[30..] { return None; }
        if !bits[..10].iter().all(|&b| b)
            || !(0..10).all(|i| bits[10 + i] == (i % 2 != 0))
            || bits[20] || bits[21] || bits[22] || bits[25] || !bits[29]
        { return None; }
        let code = ((bits[24] as usize) << 3) | ((bits[26] as usize) << 2)
            | ((bits[27] as usize) << 1) | bits[28] as usize;
        Some(Self { lapm: bits[23], qts_ucode: Self::LEVELS.get(code).copied() })
    }
}

#[cfg(test)]
mod quick_connect_tests {
    use super::*;

    #[test]
    fn digital_ack_and_qts_match_published_patterns() {
        for level in 0..4 {
            let ack = QuickConnectAck { lapm: true, answer_level: level };
            let bits = ack.to_bits().unwrap();
            assert_eq!(bits.len(), 70);
            assert_eq!(&bits[20..27], &[false, true, true, true, false, false, false]);
            assert_eq!(QuickConnectAck::from_bits(&bits), Some(ack));
            for i in 0..70 {
                let mut bad=bits.clone(); bad[i]=!bad[i];
                assert!(QuickConnectAck::from_bits(&bad).is_none());
            }
        }
        assert!(QuickConnectAck { lapm: true, answer_level: 4 }.to_bits().is_none());
        for law in [crate::v90::ucode::Law::Mu, crate::v90::ucode::Law::A] {
            let samples=quick_connect_timing(law, 79).unwrap();
            assert_eq!(samples.len(), 816);
            let v=crate::v90::ucode::level(law,79);
            let z=crate::v90::ucode::level(law,0);
            assert_eq!(&samples[..6], &[v,z,v,-v,-z,-v]);
            assert_eq!(&samples[768..774], &[-v,-z,-v,v,z,v]);
        }
        assert!(quick_connect_timing(crate::v90::ucode::Law::Mu,64).is_none());
    }

    #[test]
    fn request_has_table2_pattern_and_rejects_damaged_repetition() {
        let request = QuickConnectRequest { lapm: true, qts_ucode: Some(79) };
        let bits = request.to_bits().unwrap();
        assert_eq!(&bits[20..30], &[false, false, false, true, true, false, false, true, false, true]);
        assert_eq!(QuickConnectRequest::from_bits(&bits), Some(request));
        for i in 0..bits.len() {
            let mut damaged = bits.clone(); damaged[i] = !damaged[i];
            assert!(QuickConnectRequest::from_bits(&damaged).is_none());
        }
        for level in QuickConnectRequest::LEVELS.iter().copied().map(Some).chain([None]) {
            for lapm in [false, true] {
                let request = QuickConnectRequest { lapm, qts_ucode: level };
                assert_eq!(QuickConnectRequest::from_bits(&request.to_bits().unwrap()), Some(request));
            }
        }
        assert!(QuickConnectRequest { lapm: true, qts_ucode: Some(64) }.to_bits().is_none());
    }
}

/// Digital-side V.8 Quick Connect acknowledgement (Table 13).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuickConnectAck {
    pub lapm: bool,
    /// Table 13 LM code: 0=-9.5, 1=-12, 2=-15, 3=-18 dBm0.
    pub answer_level: u8,
}

impl QuickConnectAck {
    pub fn to_bits(self) -> Option<Vec<bool>> {
        if self.answer_level > 3 { return None; }
        let field=[false,true,true,self.lapm,false,false,false,
            self.answer_level & 2 != 0,self.answer_level & 1 != 0,true];
        let mut bits=Vec::with_capacity(70);
        for _ in 0..2 {
            bits.extend([true;10]);
            bits.extend((0..10).map(|i| i%2!=0));
            bits.extend(field);
        }
        bits.extend([true;10]);
        Some(bits)
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        if bits.len()!=70 || bits[..30]!=bits[30..60]
            || !bits[..10].iter().all(|&b| b) || !bits[60..].iter().all(|&b| b)
            || !(0..10).all(|i| bits[10+i]==(i%2!=0))
            || bits[20] || !bits[21] || !bits[22]
            || bits[24..27].iter().any(|&b| b) || !bits[29]
        { return None; }
        Some(Self { lapm: bits[23], answer_level: ((bits[27] as u8)<<1) | bits[28] as u8 })
    }
}

/// QTS then QTS-bar: 768+48 symbols at 8000 symbols/s (8.3.6).
/// Caller-requested UQTS must come from Table 2, not arbitrary PCM levels.
pub fn quick_connect_timing(law: crate::v90::ucode::Law, ucode: u8) -> Option<Vec<f64>> {
    if !QuickConnectRequest::LEVELS.contains(&ucode) { return None; }
    let v=crate::v90::ucode::level(law,ucode);
    let z=crate::v90::ucode::level(law,0);
    let pattern=[v,z,v,-v,-z,-v];
    let mut samples=Vec::with_capacity(816);
    for _ in 0..128 { samples.extend(pattern); }
    for _ in 0..8 { samples.extend(pattern.map(|sample| -sample)); }
    Some(samples)
}

fn frame(payload: &[bool]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(payload.len() + 32);
    bits.extend(info::FILL);
    bits.extend(info::SYNC);
    bits.extend_from_slice(payload);
    let crc = info::crc(payload);
    bits.extend((0..16).map(|i| crc & (1 << i) != 0));
    bits.extend(info::FILL);
    bits
}

/// Analogue initial capabilities: V.92 repurposes the V.34 clock bits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalogueCapabilities {
    pub legacy: Info0,
    pub v92: bool,
    pub short_phase2: bool,
}

impl AnalogueCapabilities {
    pub fn to_bits(self) -> Vec<bool> {
        let legacy = self.legacy.to_bits();
        let mut payload = info::unframe(&legacy, info::INFO0_BITS).unwrap().to_vec();
        payload[14] = self.v92;
        payload[15] = self.short_phase2;
        frame(&payload)
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let payload = info::unframe(bits, info::INFO0_BITS)?;
        let mut legacy = Info0::from_bits(bits)?;
        legacy.clock = 0;
        Some(Self { legacy, v92: payload[14], short_phase2: payload[15] })
    }
}

/// Digital initial capabilities. The two flags have the opposite order
/// to INFO0a; accepting the analogue layout here would mis-negotiate V.92.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DigitalCapabilities {
    pub legacy: Info0d,
    pub v92: bool,
    pub short_phase2: bool,
}

impl DigitalCapabilities {
    pub fn to_bits(self) -> Vec<bool> {
        let legacy = self.legacy.to_bits();
        let mut payload = info::unframe(&legacy, info::INFO0D_BITS).unwrap().to_vec();
        payload[14] = self.short_phase2;
        payload[15] = self.v92;
        frame(&payload)
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let payload = info::unframe(bits, info::INFO0D_BITS)?;
        Some(Self { legacy: Info0d::from_bits(bits)?, v92: payload[15], short_phase2: payload[14] })
    }
}

/// Full-phase digital probing results. Bit 70 is PCM-upstream support in
/// V.92, rather than the V.34 3429-symbol carrier flag. A receiver which
/// cannot perform PCM upstream must explicitly clear it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DigitalProbe {
    pub legacy: Info1c,
    pub pcm_upstream: bool,
}

impl DigitalProbe {
    pub fn to_bits(self) -> Vec<bool> {
        let legacy = self.legacy.to_bits();
        let mut payload = info::unframe(&legacy, info::INFO1C_BITS).unwrap().to_vec();
        payload[70 - 12] = self.pcm_upstream;
        frame(&payload)
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let payload = info::unframe(bits, info::INFO1C_BITS)?;
        let mut legacy = Info1c::from_bits(bits)?;
        legacy.probed[5].high_carrier = false;
        Some(Self { legacy, pcm_upstream: payload[70 - 12] })
    }
}

fn put_lsb(payload: &mut [bool], at: usize, width: usize, value: u16) {
    for i in 0..width { payload[at + i] = value & (1 << i) != 0; }
}

fn get_lsb(payload: &[bool], at: usize, width: usize) -> u16 {
    (0..width).fold(0, |v, i| v | (u16::from(payload[at + i]) << i))
}

/// INFO1a selecting 8000 symbols/s in both directions (Table 18). This
/// is an explicit PCM request; it must not be parsed as V.90 QAM upstream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PcmUpstreamRequest {
    pub filter_sections: u8,
    pub total_coefficient_limit: u16,
    pub section_coefficient_limit: u16,
    pub md_units: u8,
    pub uinfo: u8,
}

impl PcmUpstreamRequest {
    pub fn to_bits(self) -> Option<Vec<bool>> {
        if self.filter_sections > 3 || self.md_units > 127 || !(67..=127).contains(&self.uinfo) {
            return None;
        }
        let total = [192, 256, 320, 384].iter().position(|&n| n == self.total_coefficient_limit)?;
        let section = [128, 192, 256, 320].iter().position(|&n| n == self.section_coefficient_limit)?;
        let mut payload = vec![false; 38];
        put_lsb(&mut payload, 0, 2, u16::from(self.filter_sections));
        put_lsb(&mut payload, 2, 2, total as u16);
        put_lsb(&mut payload, 4, 2, section as u16);
        put_lsb(&mut payload, 6, 7, u16::from(self.md_units));
        put_lsb(&mut payload, 13, 7, u16::from(self.uinfo));
        put_lsb(&mut payload, 22, 3, 6);
        put_lsb(&mut payload, 25, 3, 6);
        payload[28..].fill(true);
        Some(frame(&payload))
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let payload = info::unframe(bits, info::INFO1A_BITS)?;
        if get_lsb(payload, 22, 3) != 6 || get_lsb(payload, 25, 3) != 6 { return None; }
        let uinfo = get_lsb(payload, 13, 7) as u8;
        if uinfo <= 66 { return None; }
        Some(Self {
            filter_sections: get_lsb(payload, 0, 2) as u8,
            total_coefficient_limit: 192 + 64 * get_lsb(payload, 2, 2),
            section_coefficient_limit: 128 + 64 * get_lsb(payload, 4, 2),
            md_units: get_lsb(payload, 6, 7) as u8,
            uinfo,
        })
    }
}

/// Hold timeout values are encoded most-significant bit first, unlike the
/// numbered LSB:MSB integer fields in initial capabilities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldTimeout { Seconds(u16), Unlimited }

const TIMEOUTS: [u16; 12] = [10, 20, 30, 40, 60, 120, 180, 240, 360, 480, 720, 960];

impl HoldTimeout {
    fn code(self) -> Option<u8> {
        match self {
            Self::Unlimited => Some(13),
            Self::Seconds(seconds) => TIMEOUTS.iter().position(|&s| s == seconds).map(|n| n as u8 + 1),
        }
    }
    fn decode(code: u8) -> Option<Self> {
        match code {
            1..=12 => Some(Self::Seconds(TIMEOUTS[usize::from(code - 1)])),
            13 => Some(Self::Unlimited),
            _ => None,
        }
    }
}

/// How a modem-on-hold exchange (9.10) left the call, at the held end.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Held {
    /// MHreq granted with MHack: ANSam for `seconds` (T1, 9.10.2.1), then
    /// phase 1 again as the answer modem if the caller comes back.
    OnHold { seconds: f64 },
    /// MHfrr (9.10.2.3): silence up to 80 ms, ANSam, phase 1 as the answer
    /// modem.
    FastReconnect,
    /// MHclrd answered with MHcda (9.10.2.2): the call is over.
    ClearedDown,
}

impl HoldTimeout {
    /// T1 in seconds; no limit is forever.
    pub fn seconds(self) -> f64 {
        match self {
            Self::Seconds(s) => f64::from(s),
            Self::Unlimited => f64::INFINITY,
        }
    }

    /// The T1 this end grants: V92_HOLD_T1 in seconds (one of Table 33's,
    /// or "unlimited"), else four minutes.
    pub fn granted() -> Self {
        match std::env::var("V92_HOLD_T1").ok().as_deref().map(str::trim) {
            Some("unlimited") => Self::Unlimited,
            Some(v) => v.parse::<u16>().ok().map(Self::Seconds)
                .filter(|t| t.code().is_some()).unwrap_or(Self::Seconds(240)),
            None => Self::Seconds(240),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClearReason { IncomingCall, OutgoingCall, Other }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldMessage {
    Request,
    Acknowledge(HoldTimeout),
    Deny,
    Clear(ClearReason),
    ClearAcknowledge,
    FastReconnect,
}

impl HoldMessage {
    pub const BITS: usize = 40;

    pub fn to_bits(self) -> Option<Vec<bool>> {
        let (signal, information) = match self {
            Self::Request => (3, 3),
            Self::Acknowledge(timeout) => (5, timeout.code()?),
            Self::Deny => (7, 7),
            Self::Clear(reason) => (9, match reason {
                ClearReason::IncomingCall => 5,
                ClearReason::OutgoingCall => 6,
                ClearReason::Other => 10,
            }),
            Self::ClearAcknowledge => (11, 11),
            Self::FastReconnect => (13, 13),
        };
        let payload: Vec<_> = [signal, information].into_iter()
            .flat_map(|value| (0..4).rev().map(move |i| value & (1 << i) != 0)).collect();
        Some(frame(&payload))
    }

    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        let payload = info::unframe(bits, Self::BITS)?;
        let nibble = |bits: &[bool]| bits.iter().fold(0u8, |v, &b| (v << 1) | u8::from(b));
        let signal = nibble(&payload[..4]);
        let information = nibble(&payload[4..]);
        match (signal, information) {
            (3, 3) => Some(Self::Request),
            (5, code) => HoldTimeout::decode(code).map(Self::Acknowledge),
            (7, 7) => Some(Self::Deny),
            (9, 5) => Some(Self::Clear(ClearReason::IncomingCall)),
            (9, 6) => Some(Self::Clear(ClearReason::OutgoingCall)),
            (9, 10) => Some(Self::Clear(ClearReason::Other)),
            (11, 11) => Some(Self::ClearAcknowledge),
            (13, 13) => Some(Self::FastReconnect),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v92_capabilities_use_side_specific_bits_and_preserve_legacy_power() {
        let a = AnalogueCapabilities { legacy: Info0::default(), v92: true, short_phase2: false };
        let bits = a.to_bits();
        assert_eq!(&bits[26..28], &[true, false]);
        assert_eq!(AnalogueCapabilities::from_bits(&bits), Some(a));
        let d = DigitalCapabilities { legacy: Info0d { nominal_power: 9, ..Info0d::default() }, v92: true, short_phase2: false };
        let bits = d.to_bits();
        assert_eq!(&bits[26..28], &[false, true]);
        assert_eq!(DigitalCapabilities::from_bits(&bits), Some(d));
        let mut corrupted = bits;
        corrupted[27] = false;
        assert!(DigitalCapabilities::from_bits(&corrupted).is_none());
    }

    #[test]
    fn hold_wire_patterns_and_reserved_values_are_checked() {
        let bits = HoldMessage::Request.to_bits().unwrap();
        assert_eq!(&bits[12..20], &[false, false, true, true, false, false, true, true]);
        assert_eq!(HoldMessage::from_bits(&bits[..36]), Some(HoldMessage::Request));
        for timeout in TIMEOUTS.into_iter().map(HoldTimeout::Seconds).chain([HoldTimeout::Unlimited]) {
            let message = HoldMessage::Acknowledge(timeout);
            assert_eq!(HoldMessage::from_bits(&message.to_bits().unwrap()), Some(message));
        }
        for code in [0u8, 14, 15] {
            let mut payload = vec![false, true, false, true];
            payload.extend((0..4).rev().map(|i| code & (1 << i) != 0));
            assert!(HoldMessage::from_bits(&frame(&payload)).is_none());
        }
        assert!(HoldMessage::Acknowledge(HoldTimeout::Seconds(11)).to_bits().is_none());
        let mut corrupted = bits;
        corrupted[20] = !corrupted[20];
        assert!(HoldMessage::from_bits(&corrupted).is_none());
    }

    #[test]
    fn pcm_upstream_is_explicit_and_cannot_be_confused_with_v90() {
        let mut legacy = Info1c::default();
        legacy.probed[5].high_carrier = true;
        let bits = DigitalProbe { legacy, pcm_upstream: false }.to_bits();
        assert!(!bits[70], "a legacy carrier flag must not advertise PCM upstream");
        assert!(!DigitalProbe::from_bits(&bits).unwrap().pcm_upstream);
        let request = PcmUpstreamRequest {
            filter_sections: 3, total_coefficient_limit: 384,
            section_coefficient_limit: 320, md_units: 12, uinfo: 95,
        };
        let bits = request.to_bits().unwrap();
        assert_eq!(bits.len(), 70);
        assert_eq!(&bits[34..40], &[false, true, true, false, true, true]);
        assert!(bits[40..50].iter().all(|&b| b));
        assert_eq!(PcmUpstreamRequest::from_bits(&bits), Some(request));
        assert!(info::Info1aPcm::from_bits(&bits).is_none(), "V.90 must not accept PCM upstream as QAM");
        assert!(PcmUpstreamRequest { uinfo: 66, ..request }.to_bits().is_none());
        assert!(PcmUpstreamRequest { total_coefficient_limit: 200, ..request }.to_bits().is_none());
    }

    #[test]
    fn hold_sequences_survive_carrier_and_symbol_offsets_on_both_sides() {
        use crate::v34::dpsk::{Receiver, Side, Transmitter};
        for fs in [8000.0, 16000.0] {
            for side in [Side::Call, Side::Answer] {
                for offset in [0, 1, 7, 13] {
                    let message = HoldMessage::Acknowledge(HoldTimeout::Seconds(120));
                    let mut tx = Transmitter::new(side, fs);
                    tx.send(&message.to_bits().unwrap());
                    let mut rx = Receiver::hold_sequences(side, fs);
                    let mut initial_rx = Receiver::new(side, fs);
                    let mut received = Vec::new();
                    for n in 0..(fs as usize / 5 + offset) {
                        let sample = if n < offset { 0.0 } else { 0.3 * tx.next_sample() };
                        if let Some(message) = rx.feed_hold(sample) { received.push(message); }
                        assert!(initial_rx.feed(sample).is_none(), "MH must not be accepted as INFO");
                    }
                    assert_eq!(received, vec![message], "fs={fs}, side={side:?}, offset={offset}");
                }
            }
        }
    }

    fn lsb(b: &[bool], at: usize, width: usize) -> u16 {
        (0..width).fold(0, |v, i| v | u16::from(b[at + i]) << i)
    }

    #[test]
    fn table_18_bits_by_position() {
        let r = PcmUpstreamRequest { filter_sections: 3, total_coefficient_limit: 384,
            section_coefficient_limit: 128, md_units: 0, uinfo: 100 };
        let b = r.to_bits().unwrap();
        assert_eq!(b.len(), 70);
        assert!(b[0..4].iter().all(|&x| x) && b[66..70].iter().all(|&x| x));
        assert_eq!(b[4..12], [false, true, true, true, false, false, true, false]);
        assert!(b[12] && b[13]);
        assert!(b[14] && b[15]);
        assert!(!b[16] && !b[17]);
        assert_eq!(lsb(&b, 18, 7), 0);
        assert_eq!(lsb(&b, 25, 7), 100);
        assert!(!b[32] && !b[33]);
        assert_eq!(lsb(&b, 34, 3), 6);
        assert_eq!(lsb(&b, 37, 3), 6);
        assert!(b[40..50].iter().all(|&x| x));
    }

    #[test]
    fn table_18_is_never_read_as_table_19_or_v90() {
        let b = PcmUpstreamRequest { filter_sections: 0, total_coefficient_limit: 192,
            section_coefficient_limit: 128, md_units: 4, uinfo: 80 }.to_bits().unwrap();
        assert!(ShortQamRequest::from_bits(&b).is_none());
        assert!(info::Info1aPcm::from_bits(&b).is_none());
        assert!(PcmUpstreamRequest::from_bits(&b).is_some());
    }

    #[test]
    fn info1d_bit_70_is_pcm_upstream() {
        let legacy = Info1c::default();
        assert!(DigitalProbe { legacy, pcm_upstream: true }.to_bits()[70]);
        assert!(!DigitalProbe { legacy, pcm_upstream: false }.to_bits()[70]);
    }
}
