//! Offline inspection of an external V.90 upstream point capture.
//! V90_POINT_CAPTURE names `now re im` rows starting at B1.
use datapump::v34::{data::{Decoder, Params}, frame::Framing, info::SymbolRate, trellis::Code};
use datapump::v32::Mode;
use dsp::Complex;
use ec::hdlc::{Decoder as Hdlc, Fcs};
#[test]
#[ignore = "requires a local receiver capture"]
fn compare_gain_and_phase_against_frame_checksums() {
 let path=std::env::var("V90_POINT_CAPTURE").expect("V90_POINT_CAPTURE required");
 let text=std::fs::read_to_string(path).unwrap();
 let points:Vec<Complex>=text.lines().filter_map(|line| {
  let mut f=line.split_whitespace(); f.next()?;
  Some(Complex::new(f.next()?.parse().ok()?,f.next()?.parse().ok()?))
 }).take(48_000).collect();
 assert!(points.len()>4_000,"capture too short");
 for gain in [0.98,0.99,1.0,1.01,1.02,1.03] {
  for phase in [-0.005,0.0,0.005] {
   let params=Params {framing:Framing::new(SymbolRate::S3200,28_800,false,false).unwrap(),code:Code::States16,nonlinear:false,precoding:[(0,0);3],mode:Mode::Answer};
   let mut data=Decoder::new(params); let mut hdlc=Hdlc::new(Fcs::Bits16); hdlc.accept_either();
   let (mut good,mut bad,mut information_bytes)=(0,0,0);
   for &point in &points {
    data.feed(point*Complex::from_polar(gain,phase));
    for bit in data.take_bits() {
     if let Some(frame)=hdlc.feed(bit){match frame {
      Ok(frame)=>{good+=1; if frame.len()>=3 && frame[1]&1==0 {information_bytes+=frame.len()-3;}},
      Err(_)=>bad+=1,
     }}
    }
   }
   println!("gain={gain:.3} phase={phase:.3} good={good} bad={bad} information_bytes={information_bytes} outside={}",data.outside());
  }
 }
}