//! Optional automatic-answer V.8bis transactions 2 and 12 (08/1996).
//! INIT/RESP denote transaction roles, not additional on-wire signals.
//! CRe -> ESr/CL -> MS -> ACK(1), or CRe -> CRd -> CL -> MS -> ACK(1).
//! Only full V.8 modem startup is selected; PCM availability is decided by V.8.

use dsp::FskDetector;
use ec::hdlc::{self,Fcs};
use v8::{Modulation,Modulations};
use crate::v8::Role;
#[cfg(test)]
use crate::bell103::Bell103Tx;

#[derive(Debug,Clone,Copy,PartialEq,Eq)]
pub enum Kind { Ms=1,Cl=2,Clr=3,Ack=4,Nak=8,Unsupported=10 }

#[derive(Debug,Clone,PartialEq,Eq)]
pub struct Message {
    pub kind:Kind,
    pub full_v8:bool,
    pub more:bool,
    pub ack:bool,
    pub modes:Modulations,
    pub lapm:bool,
    pub compression:bool,
}

// Walk unknown trees as well as recognized ones. Bit 8 terminates level 1/2
// blocks; bit 7 terminates level 2/3 groups. Never infer offsets from known bits.
fn block<'a>(bytes:&'a [u8],pos:&mut usize,mask:u8)->Option<&'a [u8]> {
    let start=*pos;
    loop {let b=*bytes.get(*pos)?; *pos+=1; if b&mask!=0 {break;}}
    Some(&bytes[start..*pos])
}
fn tree<'a>(bytes:&'a [u8],pos:&mut usize)->Option<(&'a [u8],Vec<(usize,&'a [u8])>)> {
    let np=block(bytes,pos,0x80)?;
    let sp=block(bytes,pos,0x80)?;
    let mut branches=Vec::new();
    for (octet,&value) in sp.iter().enumerate() {
        for bit in 0..7 {
            if value&(1<<bit)==0 {continue;}
            let child=block(bytes,pos,0x40)?;
            if child.last()?&0x80==0 {
                let sub=block(bytes,pos,0x40)?;
                for &v in sub {for k in 0..6 {if v&(1<<k)!=0 {block(bytes,pos,0x40)?;}}}
                // The final nested NPar(3) must close its parent Par(2).
                if bytes.get(pos.checked_sub(1)?)?&0x80==0 {return None;}
            }
            branches.push((octet*7+bit,child));
        }
    }
    Some((np,branches))
}

impl Message {
    pub fn control(kind:Kind)->Self {
        Self {kind,full_v8:false,more:false,ack:false,modes:Modulations::of(&[]),lapm:false,compression:false}
    }
    pub fn capabilities(kind:Kind,modes:Modulations,lapm:bool)->Self {
        Self {kind,full_v8:true,more:false,ack:kind==Kind::Ms,modes,lapm,compression:lapm}
    }
    pub fn encode(&self)->Vec<u8> {
        let mut out=vec![0x10|self.kind as u8];
        if !matches!(self.kind,Kind::Ms|Kind::Cl|Kind::Clr) {return out;}
        out.extend([0x80|u8::from(self.full_v8)|(u8::from(self.more)<<2)|(u8::from(self.ack)<<3),
            0x80,0x80,0x81]);
        let mut data=vec![u8::from(self.lapm)<<1|u8::from(self.compression)<<2];
        let mut second=0;
        if self.modes.contains(Modulation::V34Duplex) {second|=0x10;}
        if self.modes.contains(Modulation::V32bis) {second|=0x20;}
        data.push(second);
        let mut third=0;
        if self.modes.contains(Modulation::V22bis) {third|=2;}
        data.push(third|0xc0); out.extend(data); out
    }
    pub fn decode(bytes:&[u8])->Option<Self> {
        if bytes.is_empty() || bytes.len()>64 || bytes[0]>>4!=1 {return None;}
        let kind=match bytes[0]&15 {1=>Kind::Ms,2=>Kind::Cl,3=>Kind::Clr,4=>Kind::Ack,
            8=>Kind::Nak,10=>Kind::Unsupported,_=>return None};
        let mut msg=Self::control(kind);
        if !matches!(kind,Kind::Ms|Kind::Cl|Kind::Clr) {return (bytes.len()==1).then_some(msg);}
        let mut pos=1;
        let (id,_)=tree(bytes,&mut pos)?;
        msg.full_v8=id[0]&1!=0; msg.more=id[0]&4!=0;msg.ack=id[0]&8!=0;
        // Unsupported non-standard fields cannot be selected accidentally.
        if id[0]&0x40!=0 {return None;}
        let (standard,branches)=tree(bytes,&mut pos)?;
        if standard[0]&0x40!=0 || pos!=bytes.len() {return None;}
        for (index,data) in branches {
            if index!=0 {continue;}
            msg.lapm=data[0]&2!=0;msg.compression=data[0]&4!=0;
            if data.get(1).is_some_and(|b|b&0x10!=0) {msg.modes.insert(Modulation::V34Duplex);}
            if data.get(1).is_some_and(|b|b&0x20!=0) {msg.modes.insert(Modulation::V32bis);}
            if data.get(2).is_some_and(|b|b&2!=0) {msg.modes.insert(Modulation::V22bis);}
        }
        Some(msg)
    }
}

#[derive(Debug)]
pub struct MessageRx {
    detector:FskDetector,decoder:hdlc::Decoder,fs:f64,clock:f64,last:bool,
}
impl MessageRx {
    pub fn new(fs:f64,responding:bool)->Self {
        let (space,mark)=if responding {(1850.0,1650.0)} else {(1180.0,980.0)};
        Self {detector:FskDetector::new(space,mark,300.0,fs),
            decoder:hdlc::Decoder::new(Fcs::Bits16).with_max_octets(66).with_minimum_payload(1),fs,clock:0.0,last:true}
    }
    pub(crate) fn feed_frame(&mut self,line:f64)->Option<Result<Vec<u8>,()>> {
        let bit=self.detector.feed(line)>0.0;
        if !self.detector.carrier() {self.clock=0.0;self.last=bit;return None;}
        if bit!=self.last {self.clock=self.fs/600.0;self.last=bit;}
        self.clock-=1.0;
        if self.clock>0.0 {return None;}
        self.clock+=self.fs/300.0;
        self.decoder.feed(bit).map(|r|r.map_err(|_|()))
    }
    pub fn feed(&mut self,line:f64)->Option<Result<Message,()>> {
        self.feed_frame(line).map(|r|r.and_then(|bytes|Message::decode(&bytes).ok_or(())))
    }
}

fn message_bits(msg:&Message)->Vec<bool> {
    framed_bits(&msg.encode())
}

pub(crate) fn framed_bits(bytes:&[u8])->Vec<bool> {
    let mut enc=hdlc::Encoder::new(Fcs::Bits16);
    // 100 ms of continuous mark, two opening flags, one terminating flag.
    let mut bits=vec![true;30];enc.idle(1);enc.frame(bytes);
    while let Some(bit)=enc.next_bit() {bits.push(bit);} bits
}

#[derive(Debug,Clone,Copy,PartialEq)]
pub enum Output {Sample(f64),Start(Role),QuickConnect {lapm:bool,ucode:u8},Fallback}

/// The answering end's V.8 bis: BinModem 4's station ([`crate::v8bis_station`])
/// opening with CRe, with this server's V.92 QC2a request watched for
/// alongside it. What the station agrees is handed to V.8 as before.
#[derive(Debug)]
pub struct Answerer {
    station:crate::v8bis_station::Station,
    /// QC2a arrives as a V.8 bis-shaped frame the station does not know.
    qc:MessageRx,
    done:bool,
}
impl Answerer {
    pub fn new(fs:f64,ours:Modulations,lapm:bool)->Self {Self::offering(fs,ours,lapm,false)}
    /// `pcm`: this end is a V.90 and V.92 digital modem on a digital line.
    pub fn offering(fs:f64,ours:Modulations,lapm:bool,pcm:bool)->Self {
        use v8::bis::{Data,Identification,Kind as K,Message as M,Network,Signal};
        let data=Data {v42:lapm,v34:ours.contains(Modulation::V34Duplex),
            v32bis:ours.contains(Modulation::V32bis),v22bis:ours.contains(Modulation::V22bis),
            v90_digital:pcm,v92_digital:pcm,..Data::default()};
        let id=Identification {v8:true,ack1:true,
            network:pcm.then(||Network {digital:true,..Network::default()}),..Identification::default()};
        // One CRe: a caller with no V.8 bis is kept from its answer tone for
        // 3.4 s rather than 9.4 s.
        let station=crate::v8bis_station::Station::answering(M::data(K::Cl,id,data),Signal::CRe,fs).attempts(1);
        Self {station,qc:MessageRx::new(fs,true),done:false}
    }
    pub fn phase(&self)->&'static str {self.station.phase()}
    pub fn step(&mut self,line:f64)->Output {
        use crate::v8bis_station::Status;
        if self.done {return Output::Fallback;}
        // QC2a can start after only 50 ms of CRe: listen during transmission.
        if let Some(Ok(bytes))=self.qc.feed_frame(line) {
            if bytes.len()==2 && bytes[0]&15==13 && bytes[1]&0xd0==0 {
                let code=(bytes[1]&15).reverse_bits()>>4;
                let levels=[61,62,63,66,67,70,71,74,75,78,79,82,83,86,87];
                if let Some(&ucode)=levels.get(code as usize) {
                    self.done=true;
                    return Output::QuickConnect {lapm:bytes[1]&0x20!=0,ucode};
                }
            }
        }
        let out=self.station.step(line);
        for note in self.station.take_notes() {eprintln!("V.8bis: {note}");}
        match self.station.status() {
            Status::Running=>Output::Sample(out),
            Status::Selected(sel)=> {
                self.done=true;
                eprintln!("V.8bis: selected {:?}, {} the answering modem",sel.modulation(),
                    if sel.answer {"this end is"} else {"the far end is"});
                // 9.9: the station that received the MS answers what follows.
                Output::Start(if sel.answer {Role::Answering} else {Role::Calling})
            }
            Status::Failed(why)=> {self.done=true;eprintln!("V.8bis: gave up: {why}");Output::Fallback}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_binmodem4_caller_selects_v92_and_the_server_answers_what_follows() {
        use std::collections::VecDeque;
        use v8::bis::{Data,Identification,Kind as K,Message as M};
        use crate::v8bis_station::{OnCr,OnMr,Station,Status};
        for (delay,on_cr) in [(0.020,OnCr::Crd),(0.150,OnCr::Crd),(0.020,OnCr::Cl),(0.150,OnCr::Cl)] {
            let fs=8000.0;
            let modes=Modulations::of(&[Modulation::V34Duplex,Modulation::V32bis]);
            let mut server=Answerer::offering(fs,modes,true,true);
            let caps=M::data(K::Cl,Identification {v8:true,ack1:true,..Identification::default()},
                Data {v42:true,v34:true,v32bis:true,v22bis:true,v90_analogue:true,v92_analogue:true,..Data::default()});
            let mut caller=Station::calling(caps,fs).replying(OnMr::Ms,on_cr);
            let mut up=VecDeque::from(vec![0.0;(delay*fs) as usize]);let mut down=up.clone();
            let mut started=None;
            for _ in 0..(fs*8.0) as usize {
                let heard=up.pop_front().unwrap();
                match server.step(heard) {
                    Output::Sample(v)=>down.push_back(v),
                    Output::Start(role)=>{started=Some(role);break;}
                    other=>panic!("server gave {other:?} at delay {delay}"),
                }
                let c=caller.step(down.pop_front().unwrap());up.push_back(c);
            }
            // CRd: the caller chooses and sends MS, so this end answers.
            // CL: this end chooses and sends MS, and the caller answers.
            let want=if on_cr==OnCr::Crd {Role::Answering} else {Role::Calling};
            assert_eq!(started,Some(want),"delay {delay} {on_cr:?}");
            // The server's ACK is still on its way down the line.
            for _ in 0..(fs*1.0) as usize {
                if caller.status()!=Status::Running {break;}
                caller.step(down.pop_front().unwrap_or(0.0));
            }
            let Status::Selected(sel)=caller.status() else {panic!("caller: {:?}",caller.status())};
            assert_eq!(sel.modulation(),Some(v8::bis::Modulation::V92),"delay {delay}");
        }
    }
    #[test]
    fn published_information_field_and_crc_framing() {
        let msg=Message::capabilities(Kind::Ms,Modulations::of(&[Modulation::V34Duplex]),true);
        assert_eq!(msg.encode(),[0x11,0x89,0x80,0x80,0x81,0x06,0x10,0xc0]);
        assert_eq!(Message::decode(&msg.encode()),Some(msg));
        assert_eq!(Message::control(Kind::Ack).encode(),[0x14]);
        assert!(Message::decode(&[0x11,0x01,0x80]).is_none());
    }
    #[test]
    fn synchronous_hdlc_survives_both_v21_bands_and_sample_rates() {
        for fs in [8000.0,16000.0] {for responding in [false,true] {
            let msg=Message::capabilities(Kind::Cl,Modulations::of(&[Modulation::V34Duplex]),true);
            let (space,mark)=if responding {(1850.0,1650.0)} else {(1180.0,980.0)};
            let mut tx=Bell103Tx::with_tones(space,mark,fs);tx.push_bits(&message_bits(&msg));tx.set_transmitting(true);
            let mut rx=MessageRx::new(fs,responding);let mut decoded=None;
            for _ in 0..fs as usize {if let Some(Ok(m))=rx.feed(tx.next_sample()*0.15) {decoded=Some(m);}}
            assert_eq!(decoded,Some(msg),"fs={fs}, responding={responding}");
        }}
    }
    #[test]
    fn unanswered_cre_returns_to_existing_startup() {
        let mut a=Answerer::new(8000.0,Modulations::of(&[Modulation::V34Duplex]),true);
        let mut fallback=false;
        for _ in 0..32000 {fallback|=a.step(0.0)==Output::Fallback;}
        assert!(fallback);
    }

    #[test]
    fn cre_esr_cl_ms_ack_waveforms_handover_into_pcm_v8() {
        use std::collections::VecDeque;
        use crate::v8::{Modem,Status};
        use crate::v8bis_station::{OnCr,OnMr,Station,Status as BisStatus};
        use v8::{CallFunction,Pcm,PcmRole};
        use v8::bis::{Data,Identification,Kind as K,Message as M};
        for delay in [0.020,0.150] {
            let fs=8000.0;
            let modes=Modulations::of(&[Modulation::V34Duplex]);
            let mut server=Modem::new(Role::Answering,CallFunction::Data,modes,fs)
                .offering_pcm(Pcm {digital:true,..Pcm::default()}).offering_lapm().offering_v8bis();
            // A BinModem 4 caller: ESr and its CL, the server's MS, ACK(1),
            // then its V.8 as the answering modem: it received the MS (9.9).
            let caps=M::data(K::Cl,Identification {v8:true,ack1:true,..Identification::default()},
                Data {v42:true,v34:true,v90_analogue:true,..Data::default()});
            let mut bis=Some(Station::calling(caps,fs).replying(OnMr::Ms,OnCr::Cl));
            let mut peer=Modem::new(Role::Answering,CallFunction::Data,modes,fs)
                .offering_pcm(Pcm {analogue:true,..Pcm::default()}).offering_lapm();
            let mut upstream=VecDeque::from(vec![0.0;(delay*fs) as usize]);
            let mut downstream=upstream.clone();
            let mut last_server=0.0;let mut selected=false;
            for _ in 0..(fs*16.0) as usize {
                let incoming=downstream.pop_front().unwrap();
                let from_peer=match bis.as_mut() {
                    Some(b)=> {
                        let s=b.step(incoming);
                        match b.status() {
                            BisStatus::Running=>{}
                            BisStatus::Selected(sel)=> {assert!(sel.answer);selected=true;bis=None;}
                            other=>panic!("caller V.8 bis: {other:?} at delay {delay}"),
                        }
                        s
                    }
                    None=>0.3*peer.step(incoming),
                };
                downstream.push_back(last_server);
                let received=upstream.pop_front().unwrap();upstream.push_back(from_peer);
                last_server=0.3*server.step(received);
                if matches!(server.status(),Status::Agreed(_)) {break;}
            }
            assert!(selected,"V.8 bis did not finish at delay {delay}");
            assert_eq!(server.status(),Status::Agreed(Modulation::V34Duplex));
            assert_eq!(server.pcm_role(),Some(PcmRole::Digital));
            assert_eq!(server.training_role(),Role::Calling);
        }
    }
}

#[cfg(test)]
mod qc2_tests {
    use super::*;
    use crate::v8::Modem;
    use v8::CallFunction;
    fn modem()->Modem {Modem::new(Role::Answering,CallFunction::Data,
        Modulations::of(&[Modulation::V34Duplex]),8000.0).offering_pcm(v8::Pcm {digital:true,..v8::Pcm::default()}).offering_lapm()
        .offering_quick_connect(crate::v90::ucode::Law::Mu).offering_v8bis()}
    #[test]
    fn early_qc2_request_interrupts_cre_and_replies_on_low_band() {
        let mut server=modem();let mut tx=Bell103Tx::with_tones(1850.0,1650.0,8000.0);
        tx.push_bits(&framed_bits(&[0x2d,0x29]));tx.set_transmitting(true);
        let mut ear=MessageRx::new(8000.0,false);let mut ack=None;let mut when=None;
        for n in 0..16000 {
            let line=if n>=4000 {tx.next_sample()*0.15} else {0.0};
            let out=server.step(line);
            if server.phase()=="V.92 QCA2d" && when.is_none(){when=Some(n);}
            if let Some(Ok(bytes))=ear.feed_frame(out*0.3) {ack=Some(bytes);break;}
        }
        assert!(when.unwrap()<7200,"CRe did not stop for early QC2a");
        assert_eq!(ack,Some(vec![0x2d,0xe0]));
    }
    #[test]
    fn with_qc_in_v8bis_off_a_qc2a_gets_no_qca2d_and_ansam_follows() {
        // V92_QC_V8BIS=0: nothing goes back on the low band, and the
        // controller moves on to ANSam, where the caller sends QC1a.
        let mut server=modem().quick_connect_after_ansam_only();
        let mut tx=Bell103Tx::with_tones(1850.0,1650.0,8000.0);
        tx.push_bits(&framed_bits(&[0x2d,0x29]));tx.set_transmitting(true);
        let mut ear=MessageRx::new(8000.0,false);let mut ansam=None;
        for n in 0..32000 {
            let line=if n>=4000 {tx.next_sample()*0.15} else {0.0};
            let out=server.step(line);
            assert_ne!(server.phase(),"V.92 QCA2d","QC2a was answered at sample {n}");
            if let Some(Ok(bytes))=ear.feed_frame(out*0.3) {panic!("a V.8 bis reply went out: {bytes:02x?}");}
            if server.phase()=="ANSam" && ansam.is_none() {ansam=Some(n);}
        }
        assert!(ansam.is_some(),"ANSam never began after the unanswered QC2a");
    }
    #[test]
    #[ignore="requires the saved local failed V.8bis call"]
    fn recorded_qc2_request_is_acknowledged_instead_of_timing_out() {
        let b=std::fs::read(std::env::var("V8BIS_CAPTURE").unwrap()).unwrap();let mut p=12;
        let data=loop {let n=u32::from_le_bytes(b[p+4..p+8].try_into().unwrap()) as usize;
            if &b[p..p+4]==b"data" {break &b[p+8..p+8+n];}p+=8+n+(n&1);};
        let mut server=modem();let mut heard=false;
        for f in data.chunks_exact(4).take(12000) {
            server.step(i16::from_le_bytes([f[0],f[1]]) as f64/32768.0);
            heard|=server.phase()=="V.92 QCA2d";
        }
        assert!(heard,"recorded QC2a still ignored");
    }
    #[test]
    #[ignore = "diagnostic for a supplied stereo modem recording"]
    fn inspect_recorded_v8bis_frames() {
        let b=std::fs::read(std::env::var("V8BIS_CAPTURE").unwrap()).unwrap();
        let mut p=12;
        let data=loop {let n=u32::from_le_bytes(b[p+4..p+8].try_into().unwrap()) as usize;
            if &b[p..p+4]==b"data" {break &b[p+8..p+8+n];}p+=8+n+(n&1);};
        for channel in 0..2 {
            for high in [false,true] {
                let mut rx=MessageRx::new(8000.0,high);
                let mut valid=0;
                for (i,f) in data.chunks_exact(4).take(8*8000).enumerate() {
                    let off=channel*2;
                    let sample=i16::from_le_bytes([f[off],f[off+1]]) as f64/32768.0;
                    if let Some(frame)=rx.feed_frame(sample) {
                        if frame.is_ok(){valid+=1;}
                        eprintln!("at {:.4}s channel={channel} high={high} frame={frame:?}",i as f64/8000.0);
                    }
                }
                eprintln!("channel={channel} high={high} valid_frames={valid}");
            }
        }
    }
}
