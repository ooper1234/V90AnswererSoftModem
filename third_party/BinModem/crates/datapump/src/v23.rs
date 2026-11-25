//! ITU-T V.23 (11/1988), clauses 2 and 4: asynchronous split-speed FSK.
//! The forward channel is 600/1200 baud; the optional return channel is 75.
use crate::bell103::{Bell103Rx, Bell103Tx};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel { Forward600, Forward1200, Backward75 }
impl Channel {
    pub fn baud(self) -> f64 { match self { Self::Forward600=>600.0, Self::Forward1200=>1200.0, Self::Backward75=>75.0 } }
    /// Space, mark; ITU V.23 tables in clauses 2 and 4.
    pub fn tones(self) -> (f64,f64) { match self { Self::Forward600=>(1700.0,1300.0), Self::Forward1200=>(2100.0,1300.0), Self::Backward75=>(450.0,390.0) } }
    pub fn receiver(self, fs:f64) -> Bell103Rx {let (space,mark)=self.tones(); Bell103Rx::with_baud(space,mark,self.baud(),fs)}
    pub fn transmitter(self, fs:f64) -> Bell103Tx {let (space,mark)=self.tones(); Bell103Tx::with_baud(space,mark,self.baud(),fs)}
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction { Normal, Reversed }
impl Direction {
    pub fn channels(self,answer:bool) -> (Channel,Channel) {
        if answer == (self==Self::Normal) {(Channel::Forward1200,Channel::Backward75)}
        else {(Channel::Backward75,Channel::Forward1200)}
    }
}

/// Split-speed asynchronous modem. Carrier qualification suppresses startup
/// discriminator transients; sustained carrier loss terminates the call.
#[derive(Debug)]
pub struct Modem {
    tx:Bell103Tx, rx:Bell103Rx, fs:f64, held:usize, missing:usize,
    connected:bool, lost:bool, rates:(u32,u32), received:Vec<bool>, framing:crate::framing::AsyncBits,
    bell103:bool,
    /// V.21: 300 bit/s both ways, as Bell 103 but on V.21's channels.
    v21:bool,
    /// Hand up the line bits at the recovered clock instead of re-framed
    /// characters, for a V.42 link on top that needs the synchronous stream.
    raw:bool,
}
impl Modem {
    pub fn new(answer:bool,direction:Direction,fs:f64)->Self {
        let (tx,rx)=direction.channels(answer);
        let mut transmitter=tx.transmitter(fs);transmitter.set_transmitting(true);
        Self {tx:transmitter,rx:rx.receiver(fs),fs,held:0,missing:0,connected:false,lost:false,
            rates:(tx.baud() as u32,rx.baud() as u32),received:Vec::new(),framing:crate::framing::AsyncBits::new(8),bell103:false,v21:false,raw:false}
    }
    /// Bell 103 shares the transparent FSK/framing service, with symmetric
    /// 300-baud channels and a longer initial carrier qualification.
    pub fn new_bell103(answer:bool,fs:f64)->Self {
        let role=if answer {crate::bell103::Role::Answer} else {crate::bell103::Role::Originate};
        let mut m=Self::new(answer,Direction::Normal,fs);
        m.tx=Bell103Tx::new(role,fs);m.tx.set_transmitting(true);
        m.rx=Bell103Rx::new(role,fs);m.rates=(300,300);m.bell103=true;m
    }
    /// V.21 (300 bit/s duplex FSK). The answering modem sends channel 2
    /// (mark 1650, space 1850 Hz) and hears channel 1 (mark 980, space
    /// 1180 Hz); idle is mark, so it is sending its answer mark from the start.
    pub fn new_v21(answer:bool,fs:f64)->Self {
        let (ch2,ch1)=(crate::v21::CHANNEL_2,crate::v21::CHANNEL_1);
        let (tx,rx)=if answer {(ch2,ch1)} else {(ch1,ch2)};
        let mut m=Self::new(answer,Direction::Normal,fs);
        m.tx=Bell103Tx::with_tones(tx.0,tx.1,fs);m.tx.set_transmitting(true);
        m.rx=Bell103Rx::with_tones(rx.0,rx.1,fs);m.rates=(300,300);m.v21=true;m
    }
    pub fn connected(&self)->bool {self.connected}
    pub fn failed(&self)->bool {self.lost}
    pub fn rates(&self)->(u32,u32) {self.rates}
    pub fn phase(&self)->&'static str {
        if self.v21 {
            if self.lost {"V.21 carrier lost"} else if self.connected {"V.21 data / transparent"} else {"V.21 carrier acquisition"}
        } else if self.bell103 {
            if self.lost {"Bell 103 carrier lost"} else if self.connected {"Bell 103 data / transparent"} else {"Bell 103 carrier acquisition"}
        } else if self.lost {"V.23 carrier lost"} else if self.connected {"V.23 data / transparent"} else {"V.23 carrier acquisition"}
    }
    pub fn pending_bits(&self)->usize {self.tx.pending_bits()}
    pub fn push_bits(&mut self,bits:&[bool]) {self.tx.push_bits(bits);}
    pub fn take_bits(&mut self)->Vec<bool> {std::mem::take(&mut self.received)}
    pub fn set_raw_bits(&mut self,on:bool) {self.raw=on;}
    pub fn step(&mut self,input:f64)->f64 {
        if self.lost {return 0.0;}
        let mut byte=self.rx.feed(input);
        if self.rx.carrier() {
            self.missing=0;self.held+=1;
            if !self.connected && self.held >= (self.fs*if self.bell103 || self.v21 {0.200} else {0.080}) as usize {
                self.rx.reset_framing();self.connected=true;
                byte=None;
            }
        } else {
            self.held=0;self.missing+=1;
            if self.connected && self.missing >= (self.fs*0.150) as usize {
                self.connected=false;self.lost=true;self.received.clear();
            }
        }
        let raw=self.rx.take_raw_bit();
        if self.connected {
            if self.raw {if let Some(bit)=raw {self.received.push(bit);}}
            else if let Some(byte)=byte {self.received.extend(self.framing.encode(byte));}
        }
        self.tx.next_sample()*0.3
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framing::AsyncBits;
    #[test]
    #[ignore="requires a saved V.23 hardware capture"]
    fn compare_v23_capture_receiver_bandwidths() {
        let raw=std::fs::read(std::env::var("BELL_CAPTURE").unwrap()).unwrap();
        let base=std::env::var("FSK_BYTES").unwrap();
        for bandwidth in [40,50,60,65,70,75,80,85,90,100,110,125,150] {
            let mut rx=Bell103Rx::capture_bandwidth(450.0,390.0,75.0,8000.0,bandwidth as f64);
            let mut bytes=Vec::new();
            for p in raw.chunks_exact(2) {
                if let Some(b)=rx.feed(i16::from_le_bytes([p[0],p[1]]) as f64/32768.0) {bytes.push(b);}
            }
            eprintln!("bandwidth={bandwidth} bytes={} framing_errors={}",bytes.len(),rx.framing_errors());
            std::fs::write(format!("{base}.{bandwidth}"),bytes).unwrap();
        }
    }
    #[test]
    #[ignore="requires a saved Bell 103 hardware capture"]
    fn compare_bell103_capture_receiver_bandwidths() {
        let raw=std::fs::read(std::env::var("BELL_CAPTURE").unwrap()).unwrap();
        let base=std::env::var("FSK_BYTES").unwrap();
        for bandwidth in [150,175,200,225,250,275,300,325,350,375,400,450,500] {
            let mut rx=Bell103Rx::capture_bandwidth(1070.0,1270.0,300.0,8000.0,bandwidth as f64);
            let mut bytes=Vec::new();
            for p in raw.chunks_exact(2) {
                if let Some(b)=rx.feed(i16::from_le_bytes([p[0],p[1]]) as f64/32768.0) {bytes.push(b);}
            }
            eprintln!("bandwidth={bandwidth} bytes={} framing_errors={}",bytes.len(),rx.framing_errors());
            std::fs::write(format!("{base}.{bandwidth}"),bytes).unwrap();
        }
    }
    #[test]
    #[ignore="requires a saved Bell 103 hardware capture"]
    fn compare_bell103_capture_symbol_rates() {
        let raw=std::fs::read(std::env::var("BELL_CAPTURE").unwrap()).unwrap();
        let base=std::env::var("FSK_BYTES").unwrap();
        for baud in [110,150,200,250,280,290,295,300,305,310,320,350,400,600] {
            let mut rx=Bell103Rx::capture_bandwidth(1070.0,1270.0,baud as f64,8000.0,300.0);
            let mut bytes=Vec::new();
            for p in raw.chunks_exact(2) {
                if let Some(b)=rx.feed(i16::from_le_bytes([p[0],p[1]]) as f64/32768.0) {bytes.push(b);}
            }
            eprintln!("baud={baud} bytes={} framing_errors={} flags={}",bytes.len(),rx.framing_errors(),bytes.iter().filter(|&&b|b==0x7e).count());
            std::fs::write(format!("{base}.baud{baud}"),bytes).unwrap();
        }
    }
    #[test]
    #[ignore = "requires a saved hardware capture"]
    fn replay_fsk_hardware_receiver() {
        let raw=std::fs::read(std::env::var("BELL_CAPTURE").expect("BELL_CAPTURE")).unwrap();
        let mut rx=if std::env::var_os("V23_CAPTURE").is_some() {Channel::Backward75.receiver(8000.0)}
            else {Bell103Rx::new(crate::bell103::Role::Answer,8000.0)};
        let mut missing=0;let mut max_missing=0;let mut bytes=Vec::new();
        for (n,p) in raw.chunks_exact(2).enumerate() {
            if let Some(b)=rx.feed(i16::from_le_bytes([p[0],p[1]]) as f64/32768.0) {bytes.push(b);}
            if rx.carrier() {missing=0;} else {missing+=1;}
            if n>2000 {max_missing=max_missing.max(missing);}
            if missing==1200 {eprintln!("carrier absent at {:.3}s, envelope={}",n as f64/8000.0,rx.amplitude());}
        }
        eprintln!("max dropout={} samples, bytes={}, framing errors={}",max_missing,bytes.len(),rx.framing_errors());
        if let Some(path)=std::env::var_os("FSK_BYTES") {std::fs::write(path,&bytes).unwrap();}
        assert!(!bytes.is_empty(),"hardware capture contains no decoded bytes");
    }
    #[test]
    fn published_tones_and_complementary_directions() {
        assert_eq!(Channel::Forward600.tones(),(1700.0,1300.0));
        assert_eq!(Channel::Forward1200.tones(),(2100.0,1300.0));
        assert_eq!(Channel::Backward75.tones(),(450.0,390.0));
        for direction in [Direction::Normal,Direction::Reversed] {
            let (tx,rx)=direction.channels(true);let (peer_tx,peer_rx)=direction.channels(false);
            assert_eq!(tx,peer_rx);assert_eq!(rx,peer_tx);
        }
    }
    #[test]
    fn independently_clocked_noisy_fsk_recovers_bytes() {
        let payload=[0x00,0xff,0x55,0xaa,0x7e,0x81,0x13,0x42];
        for fs in [8000.0,16000.0] {
            for channel in [Channel::Forward600,Channel::Forward1200,Channel::Backward75] {
                for ppm in [-500.0,500.0] {
                    let mut tx=channel.transmitter(fs*(1.0+ppm/1e6));
                    let mut rx=channel.receiver(fs);tx.set_transmitting(true);
                    let mut recovered=Vec::new();let mut framing=AsyncBits::new(8);
                    let warm=(fs*0.2) as usize;
                    let mut rng=0x12345678u32;
                    for n in 0..warm+(fs/channel.baud()*100.0) as usize {
                        if n==warm {recovered.clear(); for byte in payload {tx.push_bits(&framing.encode(byte));}}
                        rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
                        let noise=(rng as f64/u32::MAX as f64-0.5)*0.004;
                        if let Some(byte)=rx.feed(tx.next_sample()*0.15+noise) {recovered.push(byte);}
                    }
                    assert_eq!(recovered,payload,"channel={channel:?}, fs={fs}, ppm={ppm}");
                }
            }
        }
    }
    #[test]
    fn characteristic_frequency_tolerances_recover_forward_and_return_bytes() {
        let payload=[0x00,0xff,0x55,0xaa,0x7e,0x81,0x13,0x42];
        for channel in [Channel::Forward600,Channel::Forward1200,Channel::Backward75] {
            let tolerance=if channel==Channel::Backward75 {6.0} else {16.0};
            for offset in [-tolerance,tolerance] {
                let (space,mark)=channel.tones();
                let mut tx=Bell103Tx::with_baud(space+offset,mark+offset,channel.baud(),8000.0);
                let mut rx=channel.receiver(8000.0);tx.set_transmitting(true);
                let framing=AsyncBits::new(8);let mut bytes=Vec::new();
                for n in 0..1600+(8000.0/channel.baud()*100.0) as usize {
                    if n==1600 {bytes.clear();for byte in payload {tx.push_bits(&framing.encode(byte));}}
                    if let Some(byte)=rx.feed(tx.next_sample()*0.15) {bytes.push(byte);}
                }
                assert_eq!(bytes,payload,"channel={channel:?}, offset={offset}");
            }
        }
    }
    #[test]
    fn split_speed_streams_in_both_directions_and_loses_carrier() {
        for direction in [Direction::Normal,Direction::Reversed] {
            let mut a=Modem::new(true,direction,8000.0);
            let mut b=Modem::new(false,direction,8000.0);
            let framing=AsyncBits::new(8);
            let bytes=[0x00,0xff,0x55,0xaa,0x7e];
            let bits:Vec<_>=bytes.into_iter().flat_map(|v|framing.encode(v)).collect();
            let mut to_a=0.0;let mut to_b=0.0;
            let mut ar=Vec::new();let mut br=Vec::new();
            for n in 0..16000 {
                if n==2000 {assert!(a.connected()&&b.connected());a.push_bits(&bits);b.push_bits(&bits);}
                let at=a.step(to_a);let bt=b.step(to_b);
                to_a=bt;to_b=at;
                ar.extend(a.take_bits());br.extend(b.take_bits());
            }
            assert_eq!(ar,bits,"answer direction={direction:?}");
            assert_eq!(br,bits,"caller direction={direction:?}");
            for _ in 0..8000 {a.step(0.0);b.step(0.0);}
            assert!(!a.connected()&&!b.connected());
            assert!(a.failed()&&b.failed());
        }
    }
    #[test]
    fn v21_streams_both_directions_on_its_own_channels() {
        let mut a=Modem::new_v21(true,8000.0);
        let mut b=Modem::new_v21(false,8000.0);
        let framing=AsyncBits::new(8);
        let bits:Vec<_>=[0x00,0xff,0x55,0xaa,0x7e,0x13].into_iter().flat_map(|v|framing.encode(v)).collect();
        let (mut ai,mut bi)=(0.0,0.0);let (mut ar,mut br)=(Vec::new(),Vec::new());
        for n in 0..16000 {
            if n==3000 {assert!(a.connected()&&b.connected());a.push_bits(&bits);b.push_bits(&bits);}
            let at=a.step(ai);let bt=b.step(bi);ai=bt;bi=at;
            ar.extend(a.take_bits());br.extend(b.take_bits());
        }
        assert_eq!(ar,bits);assert_eq!(br,bits);assert_eq!(a.rates(),(300,300));
        assert_eq!(a.phase(),"V.21 data / transparent");
        for _ in 0..8000 {a.step(0.0);b.step(0.0);}
        assert!(a.failed()&&b.failed());
    }
    #[test]
    fn bell103_streams_both_directions_and_loses_carrier() {
        let mut a=Modem::new_bell103(true,8000.0);
        let mut b=Modem::new_bell103(false,8000.0);
        let framing=AsyncBits::new(8);
        let bits:Vec<_>=[0x00,0xff,0x55,0xaa,0x7e,0x13].into_iter().flat_map(|v|framing.encode(v)).collect();
        let (mut ai,mut bi)=(0.0,0.0);let (mut ar,mut br)=(Vec::new(),Vec::new());
        for n in 0..16000 {
            if n==3000 {assert!(a.connected()&&b.connected());a.push_bits(&bits);b.push_bits(&bits);}
            let at=a.step(ai);let bt=b.step(bi);ai=bt;bi=at;
            ar.extend(a.take_bits());br.extend(b.take_bits());
        }
        assert_eq!(ar,bits);assert_eq!(br,bits);assert_eq!(a.rates(),(300,300));
        assert!(a.phase().starts_with("Bell 103"));
        for _ in 0..8000 {a.step(0.0);b.step(0.0);}
        assert!(a.failed()&&b.failed());
    }
    #[test]
    fn bell103_raw_mode_hands_up_a_synchronous_stream_unframed() {
        // HDLC flags and data back to back, no start or stop bits: what V.42
        // sends on the carrier. The answerer must hand them up as they came.
        let mut a=Modem::new_bell103(true,8000.0);
        let mut b=Modem::new_bell103(false,8000.0);
        a.set_raw_bits(true);
        let bits:Vec<bool>=[0x7eu8,0x7e,0x03,0x00,0x00,0xff,0x3f,0xa5,0x7e,0x7e].iter()
            .flat_map(|&v|(0..8).map(move|i|v>>i&1!=0)).collect();
        let (mut ai,mut bi)=(0.0,0.0);let mut ar=Vec::new();
        for n in 0..12000 {
            if n==3000 {assert!(a.connected());let _=a.take_bits();ar.clear();b.push_bits(&bits);}
            let at=a.step(ai);let bt=b.step(bi);ai=bt;bi=at;
            if n>=3000 {ar.extend(a.take_bits());}
        }
        // Idle mark around it; the stream itself must appear intact.
        let found=ar.windows(bits.len()).any(|w|w==bits.as_slice());
        assert!(found,"stream not recovered: {} bits",ar.len());
    }
}

#[cfg(test)]
mod raw_replay {
    use super::*;
    use crate::framing::AsyncBits;
    /// Replays channel 0 of a stereo capture (B103_CAPTURE) through the
    /// answerer's raw-bit Bell 103 receive path and prints what it deframes.
    #[test]
    #[ignore = "requires a saved Bell 103 capture"]
    fn replay_bell103_raw_receive() {
        let raw = std::fs::read(std::env::var("B103_CAPTURE").unwrap()).unwrap();
        let mut m = Modem::new_bell103(true, 8000.0);
        m.set_raw_bits(true);
        let mut de = AsyncBits::new(8);
        let mut out = Vec::new();
        for f in raw[44..].chunks_exact(4) {
            m.step(i16::from_le_bytes([f[0], f[1]]) as f64 / 32768.0);
            for b in m.take_bits() { if let Some(c) = de.feed(b) { out.push(c); } }
        }
        eprintln!("RAW {:?}", String::from_utf8_lossy(&out[..out.len().min(80)]));
    }
}
