//! Digital answering side of V.92 short Phase 1 (9.2.4).
//! Full probing remains available when shortened ranging is not negotiated.

use dsp::ToneDetector;
use crate::bell103::Bell103Tx;
use crate::v90::ucode::{self, Law};
use super::{QuickConnectAck, quick_connect_timing};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Output {
    /// No accepted QC request: continue the ordinary V.8 controller.
    Listening,
    /// FSK, requiring the usual boundary gain, or exact PCM levels.
    Sample { value: f64, pcm: bool },
    Ready { lapm: bool },
    /// TONEq did not arrive: restart ANSam and ordinary V.8 reception.
    Restart,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage { Listening, Acknowledge, Quiet, Timing, Answer, Handover, Ready }

#[derive(Debug)]
pub struct Answerer {
    fs: f64,
    law: Law,
    level: u8,
    stage: Stage,
    elapsed: u64,
    acknowledgement_samples: u64,
    after_ack: u64,
    lapm: bool,
    tx: Bell103Tx,
    sync: bool,
    previous: Option<u8>,
    timing: Vec<f64>,
    timing_position: usize,
    answer: Vec<f64>,
    answer_position: usize,
    tone: ToneDetector,
    neighbour: ToneDetector,
    tone_held: u64,
    round_trip: f64,
    delayed_reply: bool,
    v8bis_ack: bool,
    /// Whether QC2a (inside V.8 bis) is answered. When it is not, the V.8
    /// controller drops V.8 bis and plays ANSam, and a V.92 caller that hears
    /// ANSam for 1 s sends QC1a (9.2.1.1), which is still answered.
    answer_v8bis: bool,
}

impl Answerer {
    pub fn new(fs: f64, law: Law, level: u8) -> Self {
        assert!(fs >= 8000.0 && level < 4);
        Self { fs, law, level, stage: Stage::Listening, elapsed: 0,
            acknowledgement_samples: 0, after_ack: 0, lapm: false,
            tx: Bell103Tx::with_tones(1850.0,1650.0,fs), sync: false,
            previous: None, timing: Vec::new(), timing_position: 0,
            answer: answer_pcm(law,level).expect("valid level"), answer_position: 0,
            tone: ToneDetector::new(980.0,30.0,fs),
            neighbour: ToneDetector::new(1180.0,30.0,fs), tone_held: 0,
            round_trip: 0.0, delayed_reply:false, v8bis_ack:false, answer_v8bis:true }
    }

    /// Octets already decoded by the ordinary low-band V.21 receiver.
    pub fn octet(&mut self, byte: u8, lapm_supported: bool) {
        if self.stage != Stage::Listening { return; }
        if byte == 0x55 { self.sync=true; return; }
        if !std::mem::take(&mut self.sync) { self.previous=None; return; }
        // Analogue request, QC rather than QCA, and reserved bit 25 zero.
        if byte & 0x13 != 0 { self.previous=None; return; }
        if self.previous.replace(byte) != Some(byte) { return; }
        let code=((byte >> 3)&1)<<3 | ((byte >> 5)&1)<<2
            | ((byte >> 6)&1)<<1 | ((byte >> 7)&1);
        let levels=[61,62,63,66,67,70,71,74,75,78,79,82,83,86,87];
        let Some(&ucode)=levels.get(code as usize) else { self.previous=None; return; };
        self.lapm=lapm_supported && byte & 4 != 0;
        self.tx=Bell103Tx::with_tones(1850.0,1650.0,self.fs);
        self.timing_position=0; self.answer_position=0; self.tone_held=0;
        self.tone=ToneDetector::new(980.0,30.0,self.fs);
        self.neighbour=ToneDetector::new(1180.0,30.0,self.fs);
        self.timing=quick_connect_timing(self.law,ucode).expect("valid requested code");
        let bits=QuickConnectAck { lapm:self.lapm, answer_level:self.level }.to_bits().unwrap();
        self.tx.push_bits(&bits);
        self.tx.set_transmitting(true);
        // The keyer starts with one mark interval before consuming its queue.
        self.acknowledgement_samples=((bits.len()+1) as f64*self.fs/300.0).ceil() as u64;
        self.v8bis_ack=false;
        self.enter(Stage::Acknowledge);
    }

    /// QC2a (V.92 Table 3) uses synchronous V.8bis framing rather than QC1a.
    pub fn request_v8bis(&mut self,lapm:bool,ucode:u8,lapm_supported:bool) {
        if self.stage!=Stage::Listening {return;}
        if !self.answer_v8bis {
            eprintln!("V.92 QC2a received: QTS Ucode {ucode}, not answered (QC in V.8 bis is off); ANSam next, for QC1a");
            return;
        }
        let levels=[61,62,63,66,67,70,71,74,75,78,79,82,83,86,87];
        let Some(code)=levels.iter().position(|&v|v==ucode) else {return;};
        let byte=(u8::from(lapm)<<2)|(((code>>3) as u8&1)<<3)
            |(((code>>2) as u8&1)<<5)|(((code>>1) as u8&1)<<6)|((code as u8&1)<<7);
        self.octet(0x55,lapm_supported);self.octet(byte,lapm_supported);
        self.octet(0x55,lapm_supported);self.octet(byte,lapm_supported);
        if !self.active() {return;}
        self.v8bis_ack=true;
        let field=0xc0|(u8::from(self.lapm)<<5)|((self.level>>1)&1)|((self.level&1)<<1);
        let bits=crate::v8bis::framed_bits(&[0x2d,field]);
        self.tx=Bell103Tx::with_tones(1180.0,980.0,self.fs);
        self.tx.push_bits(&bits);self.tx.set_transmitting(true);
        self.acknowledgement_samples=((bits.len()+1) as f64*self.fs/300.0).ceil() as u64;
        eprintln!("V.92 QC2a received: QTS Ucode {ucode}, transmitting QCA2d");
    }

    pub fn reset_reception(&mut self) { self.sync=false; self.previous=None; }
    /// Earliest ANSpcm reply must include its outbound and inbound propagation.
    /// Before that, the caller can still be sending CM in the same 980 Hz band.
    /// Freeze the measured delay when the request is accepted.
    pub fn set_round_trip(&mut self, seconds: f64) {
        if self.stage==Stage::Listening && seconds.is_finite() {
            self.round_trip=seconds.clamp(0.0,1.5);
        }
    }
    /// Leave QC2a unanswered; QC1a after ANSam is still answered.
    pub fn decline_v8bis_requests(&mut self) {self.answer_v8bis=false;}
    /// Opt-in VoIP extension; the ITU two-second timeout is the default.
    pub fn allow_delayed_reply(&mut self) {self.delayed_reply=true;}
    fn reply_deadline(&self)->f64 {
        if self.delayed_reply {(1.0+0.075+816.0/8000.0+self.round_trip+0.350).clamp(2.0,3.0)}
        else {2.0}
    }
    pub fn active(&self) -> bool { self.stage != Stage::Listening }
    pub fn phase(&self)->&'static str {
        match self.stage {
            Stage::Listening=>"V.92 QC listening",Stage::Acknowledge=>if self.v8bis_ack {"V.92 QCA2d"} else {"V.92 QCA1d"},
            Stage::Quiet=>"V.92 QC silence",Stage::Timing=>"V.92 QTS",
            Stage::Answer=>"V.92 ANSpcm",Stage::Handover=>"V.92 QC handover",
            Stage::Ready=>"V.92 QC phase 1 complete",
        }
    }
    fn samples(&self,seconds:f64)->u64 { (seconds*self.fs).round() as u64 }
    fn enter(&mut self,stage:Stage) { self.stage=stage; self.elapsed=0; }

    pub fn step(&mut self,line:f64)->Output {
        if self.stage==Stage::Listening { return Output::Listening; }
        self.elapsed+=1;
        if matches!(self.stage,Stage::Quiet|Stage::Timing|Stage::Answer) {
            self.after_ack+=1;
            let deadline=self.reply_deadline();
            // A reply that begins before the deadline still needs detector settling and
            // the required 50 ms qualification. Bound that completion grace.
            let qualifying_reply=self.stage==Stage::Answer && self.tone_held>0
                && self.after_ack<=self.samples(deadline+0.100);
            if self.after_ack>self.samples(deadline) && !qualifying_reply {
                self.enter(Stage::Listening); self.reset_reception();
                self.tone_held=0; self.after_ack=0;
                return Output::Restart;
            }
        }
        match self.stage {
            Stage::Acknowledge => {
                let value=self.tx.next_sample();
                if self.elapsed>=self.acknowledgement_samples {
                    self.tx.set_transmitting(false); self.after_ack=0;
                    self.enter(Stage::Quiet);
                }
                Output::Sample { value,pcm:false }
            }
            Stage::Quiet => {
                if self.elapsed>=self.samples(0.075) { self.enter(Stage::Timing); }
                Output::Sample { value:0.0,pcm:true }
            }
            Stage::Timing => {
                let index=(self.timing_position as f64*8000.0/self.fs).floor() as usize;
                let value=self.timing[index]; self.timing_position+=1;
                if self.timing_position as f64*8000.0/self.fs>=self.timing.len() as f64 {
                    self.enter(Stage::Answer);
                }
                Output::Sample { value,pcm:true }
            }
            Stage::Answer => {
                self.tone.feed(line); self.neighbour.feed(line);
                let reply_possible=self.after_ack>=self.samples(
                    0.075+816.0/8000.0+self.round_trip);
                self.tone_held=if reply_possible && self.tone.amplitude()>0.008
                    && self.tone.amplitude()>3.0*self.neighbour.amplitude() {
                    self.tone_held+1
                } else {0};
                if self.tone_held>=self.samples(0.050) {
                    self.enter(Stage::Handover);
                    return Output::Sample {value:0.0,pcm:true};
                }
                let symbol=(self.answer_position as f64*8000.0/self.fs).floor() as usize;
                let sign=if (symbol/3612)%2==0 {1.0} else {-1.0};
                let value=sign*self.answer[symbol%301]; self.answer_position+=1;
                Output::Sample {value,pcm:true}
            }
            Stage::Handover => {
                if self.elapsed>=self.samples(0.075) { self.enter(Stage::Ready); }
                Output::Sample {value:0.0,pcm:true}
            }
            Stage::Ready => Output::Ready {lapm:self.lapm},
            Stage::Listening => Output::Listening,
        }
    }
}

/// Table 6 equation, quantized on the G.711 grid. Integer PCM uses the
/// 14-bit mu-law / 13-bit A-law units, converted to our 16-bit line scale.
pub fn answer_pcm(law:Law,level:u8)->Option<Vec<f64>> {
    let scale=match level {0=>1334.0,1=>1000.0,2=>708.0,3=>500.0,_=>return None};
    let scale=if law==Law::A {scale/2.0} else {scale};
    let unit=if law==Law::Mu {4.0} else {8.0};
    Some((0..301).map(|k| {
        let phase=std::f64::consts::TAU*k as f64*79.0/301.0
            +0.25*std::f64::consts::PI/301.0;
        let x=(scale*2.0f64.sqrt()*phase.cos()+0.5).floor()*unit;
        let code=(0..128u8).min_by(|&a,&b| {
            (ucode::linear(law,a) as f64-x.abs()).abs()
                .total_cmp(&(ucode::linear(law,b) as f64-x.abs()).abs())
                .then(b.cmp(&a))
        }).unwrap();
        x.signum()*ucode::level(law,code)
    }).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn v21_request_drives_live_v8_controller_to_pcm_handover() {
        use crate::v8::{Modem,Role,Status};
        use v8::{CallFunction,Modulations,Modulation,Pcm};
        for fs in [8000.0,16000.0] { for raw_only in [false,true] {
            let mut server=Modem::new(Role::Answering,CallFunction::Data,
                Modulations::of(&[Modulation::V34Duplex]),fs)
                .offering_pcm(Pcm {digital:true,..Pcm::default()})
                .offering_lapm().offering_quick_connect(Law::Mu);
            let mut tx=Bell103Tx::with_tones(1180.0,980.0,fs);
            let bits=super::super::QuickConnectRequest {lapm:true,qts_ucode:Some(79)}.to_bits().unwrap();
            tx.push_bits(&bits);
            let mut pcm_seen=false;
            for n in 0..(fs as usize*4) {
                tx.set_transmitting(n>=fs as usize && n<(fs*1.25) as usize);
                let line=if n>=(fs*2.0) as usize {0.1*(std::f64::consts::TAU*980.0*n as f64/fs).cos()}
                    else {0.15*tx.next_sample()};
                server.step_with_handshake_input(if raw_only {0.0} else {line},line);
                pcm_seen|=server.pcm_output();
                if server.quick_connected() {break;}
            }
            assert!(server.quick_connected(),"FSK QC was not accepted at {fs} Hz");
            assert!(pcm_seen);
            assert_eq!(server.status(),Status::Agreed(Modulation::V34Duplex));
            assert_eq!(server.pcm_role(),Some(v8::PcmRole::Digital));
            assert!(server.lapm());
        }}
    }
    #[test]
    fn answer_pcm_matches_independent_table7_codewords() {
        for (law,octets) in [(Law::Mu,[0xa1,0x58,0x22,0xc2,0xa3,0x38]),
            (Law::A,[0x88,0x76,0x08,0xee,0x89,0x13])] {
            let pcm=answer_pcm(law,0).unwrap();
            for (i,byte) in octets.into_iter().enumerate() {
                let (code,negative)=ucode::from_octet(law,byte);
                assert_eq!(pcm[i],ucode::level(law,code)*if negative {-1.0} else {1.0},"{law:?} index {i}");
            }
        }
    }

    fn request(a:&mut Answerer) { for byte in [0x55,0x4c,0x55,0x4c] {a.octet(byte,true);} }

    #[test]
    fn qc2a_is_answered_with_qca2d_by_default() {
        let mut a=Answerer::new(8000.0,Law::Mu,0);
        a.request_v8bis(true,78,true);
        assert!(a.active());
        assert_eq!(a.phase(),"V.92 QCA2d");
    }

    #[test]
    fn declined_qc2a_goes_unanswered_but_qc1a_after_ansam_still_works() {
        // The owner's choice for the main line (2026-11-20): Quick Connect,
        // but nothing answered inside V.8 bis. The caller then hears ANSam
        // and sends QC1a (V.92 9.2.1.1), which must still get QCA1d.
        let mut a=Answerer::new(8000.0,Law::Mu,0);
        a.decline_v8bis_requests();
        a.request_v8bis(true,78,true);
        assert!(!a.active(),"QC2a was answered");
        for _ in 0..8000 {assert_eq!(a.step(0.0),Output::Listening);}
        request(&mut a);
        assert!(a.active(),"QC1a was not answered");
        assert_eq!(a.phase(),"V.92 QCA1d");
    }

    #[test]
    fn delayed_cm_mark_cannot_end_quick_connect_before_a_reply_can_return() {
        for fs in [8000.0,16000.0] {
            for rtd in [0.150,0.725] {
                let mut a=Answerer::new(fs,Law::Mu,0);
                a.set_round_trip(rtd); request(&mut a);
                // Updating the estimate during PCM output must not move the guard.
                a.set_round_trip(0.0);
                assert_eq!(a.round_trip,rtd);
                let mut handed_over=false;
                for n in 0..(fs*2.0) as usize {
                    let t=n as f64/fs;
                    // A lingering CM mark arrives before the round trip completes.
                    // The actual reply starts only after the peer receives ANSpcm.
                    let tone=t<0.233+0.177+rtd || t>0.233+0.177+rtd+0.150;
                    let line=if tone {0.1*(std::f64::consts::TAU*980.0*t).cos()} else {0.0};
                    let out=a.step(line);
                    if a.stage==Stage::Handover || matches!(out,Output::Ready{..}) {
                        assert!(t>=0.233+0.177+rtd+0.150,
                            "accepted old mark at {t}s with RTT {rtd}s");
                        handed_over=true; break;
                    }
                }
                assert!(handed_over,"genuine reply not accepted at {fs} Hz RTT {rtd}");
            }
        }
    }

    #[test]
    #[ignore="requires BM_QC_CAPTURE pointing to the saved failed server call"]
    fn recorded_server_cm_tail_does_not_complete_quick_connect() {
        use crate::v8::{Modem,Role};
        use v8::{CallFunction,Modulations,Modulation,Pcm};
        let wav=std::fs::read(std::env::var("BM_QC_CAPTURE").unwrap()).unwrap();
        assert_eq!(&wav[..4],b"RIFF");
        let mut offset=12;
        let mut data=None;
        while offset+8<=wav.len() {
            let len=u32::from_le_bytes(wav[offset+4..offset+8].try_into().unwrap()) as usize;
            if &wav[offset..offset+4]==b"data" { data=Some(&wav[offset+8..offset+8+len]);break; }
            offset+=8+len+(len&1);
        }
        let mut server=Modem::new(Role::Answering,CallFunction::Data,
            Modulations::of(&[Modulation::V34Duplex]),8000.0)
            .offering_pcm(Pcm {digital:true,..Pcm::default()})
            .offering_lapm().offering_quick_connect(Law::Mu);
        server.set_quick_connect_round_trip(5792.0/8000.0);
        let mut accepted=false;
        for frame in data.unwrap().chunks_exact(4) {
            server.step(i16::from_le_bytes([frame[0],frame[1]]) as f64/32768.0);
            accepted|=server.phase()=="V.92 QCA1d";
            assert!(!server.quick_connected(),"recorded CM tail was accepted as TONEq");
        }
        assert!(accepted,"fixture did not exercise QC request recognition");
    }

    #[test]
    #[ignore="requires BM_QC_CAPTURE pointing to the saved failed server call"]
    fn recorded_server_late_reply_completes_quick_connect() {
        use crate::v8::{Modem,Role};
        use v8::{CallFunction,Modulations,Modulation,Pcm};
        let wav=std::fs::read(std::env::var("BM_QC_CAPTURE").unwrap()).unwrap();
        assert_eq!(&wav[..4],b"RIFF");
        let mut offset=12;
        let mut data=None;
        while offset+8<=wav.len() {
            let len=u32::from_le_bytes(wav[offset+4..offset+8].try_into().unwrap()) as usize;
            if &wav[offset..offset+4]==b"data" { data=Some(&wav[offset+8..offset+8+len]);break; }
            offset+=8+len+(len&1);
        }
        let mut server=Modem::new(Role::Answering,CallFunction::Data,
            Modulations::of(&[Modulation::V34Duplex]),8000.0)
            .offering_pcm(Pcm {digital:true,..Pcm::default()})
            .offering_lapm().offering_quick_connect(Law::Mu);
        let rtt=std::env::var("BM_QC_RTT").ok().and_then(|v|v.parse().ok()).unwrap_or(5792.0/8000.0);
        server.set_quick_connect_round_trip(rtt);
        if std::env::var("BM_QC_DELAY_COMPENSATION").as_deref()==Ok("1") {
            server=server.allowing_delayed_quick_connect_reply();
        }
        let mut accepted=false;
        let mut last_phase=server.phase();
        for (n,frame) in data.unwrap().chunks_exact(4).enumerate() {
            server.step(i16::from_le_bytes([frame[0],frame[1]]) as f64/32768.0);
            if server.phase()!=last_phase {eprintln!("capture t={} phase={} ",n as f64/8000.0,server.phase());last_phase=server.phase();}
            accepted|=server.phase()=="V.92 QCA1d";
            if server.quick_connected() { return; }
        }
        assert!(accepted,"fixture did not exercise QC request recognition");
        panic!("recorded genuine late TONEq was not accepted");
    }

    #[test]
    fn late_reply_qualification_is_bounded() {
        for fs in [8000.0,16000.0] {
            for onset in [1.960,2.010] {
                let mut a=Answerer::new(fs,Law::Mu,0); request(&mut a);
                while a.stage!=Stage::Answer { a.step(0.0); }
                let mut ready=false;
                let mut restarted=false;
                for _ in 0..(fs*3.0) as usize {
                    let line=if a.after_ack>=a.samples(onset) {
                        0.1*(std::f64::consts::TAU*980.0*a.after_ack as f64/fs).cos()
                    } else {0.0};
                    let out=a.step(line);
                    if out==Output::Restart {restarted=true;break;}
                    if matches!(out,Output::Ready{..}) {ready=true;break;}
                }
                assert_eq!(ready,onset<2.0,"onset {onset}, fs {fs}");
                assert_eq!(restarted,onset>2.0);
            }
        }
    }

    #[test]
    fn delayed_voip_reply_requires_opt_in_and_missing_reply_still_times_out() {
        for fs in [8000.0,16000.0] {for enabled in [false,true] {
            let mut a=Answerer::new(fs,Law::Mu,0);
            a.set_round_trip(0.920);
            if enabled {a.allow_delayed_reply();}
            request(&mut a);
            let mut ready=false;
            for _ in 0..(fs*4.0) as usize {
                let line=if a.after_ack>=a.samples(2.330) {
                    0.1*(std::f64::consts::TAU*980.0*a.after_ack as f64/fs).cos()
                } else {0.0};
                let out=a.step(line);
                if out==Output::Restart {break;}
                if matches!(out,Output::Ready{..}) {ready=true;break;}
            }
            assert_eq!(ready,enabled);
            let mut missing=Answerer::new(fs,Law::Mu,0);
            missing.allow_delayed_reply();missing.set_round_trip(1.5);request(&mut missing);
            assert!((0..(fs*3.3) as usize).any(|_|missing.step(0.0)==Output::Restart));
        }}
    }

    #[test]
    fn tone_reply_reaches_phase2_but_missing_reply_restarts_v8() {
        let mut a=Answerer::new(8000.0,Law::Mu,0); request(&mut a);
        let mut ready=false;
        for n in 0..16000 {
            let tone=if n>4500 {0.1*(std::f64::consts::TAU*980.0*n as f64/8000.0).cos()} else {0.0};
            if let Output::Ready {lapm}=a.step(tone) {assert!(lapm);ready=true;break;}
        }
        assert!(ready);
        let mut a=Answerer::new(8000.0,Law::Mu,0); request(&mut a);
        assert!((0..20000).any(|_| a.step(0.0)==Output::Restart));
        assert!(!a.active()); request(&mut a); assert!(a.active(),"a new request must restart all signal clocks");
    }
    #[test]
    fn quick_reply_survives_a_cancelled_receive_path() {
        use crate::v8::{Modem,Role};
        use v8::{CallFunction,Modulations,Modulation,Pcm};
        let mut server=Modem::new(Role::Answering,CallFunction::Data,
            Modulations::of(&[Modulation::V34Duplex]),8000.0)
            .offering_pcm(Pcm {digital:true,..Pcm::default()})
            .offering_lapm().offering_quick_connect(Law::Mu).offering_v8bis();
        let mut tx=Bell103Tx::with_tones(1850.0,1650.0,8000.0);
        tx.push_bits(&crate::v8bis::framed_bits(&[0x2d,0x29]));tx.set_transmitting(true);
        let mut answer_at=None;
        for n in 0..40000 {
            // Emulate an untrained canceller erasing the peer response while
            // the original line still contains a valid CRC-framed request
            // and, after ANSpcm, its 980 Hz reply.
            let raw=if n<4000 {tx.next_sample()*0.15}
                else if answer_at.is_some_and(|at|n>at+2000) {
                    (std::f64::consts::TAU*980.0*n as f64/8000.0).cos()*0.1
                } else {0.0};
            server.step_with_handshake_input(0.0,raw);
            if server.phase()=="V.92 ANSpcm" && answer_at.is_none() {answer_at=Some(n);}
            if server.quick_connected() {return;}
        }
        panic!("QC reply was lost despite a valid original line signal");
    }
}
