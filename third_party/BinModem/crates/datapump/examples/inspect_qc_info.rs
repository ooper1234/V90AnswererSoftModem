use datapump::v34::dpsk::{Receiver,Side};
fn main() {
 let wav=std::fs::read(std::env::var("BM_QC_CAPTURE").unwrap()).unwrap();
 let mut offset=12; let mut data=None;
 while offset+8<=wav.len() {
  let len=u32::from_le_bytes(wav[offset+4..offset+8].try_into().unwrap()) as usize;
  if &wav[offset..offset+4]==b"data" {data=Some(&wav[offset+8..offset+8+len]);break;}
  offset+=8+len+(len&1);
 }
 for channel in [0,1] {
  for side in [Side::Call,Side::Answer] {
   let mut rx=Receiver::new(side,8000.0);
   for (n,f) in data.unwrap().chunks_exact(4).take(8000*30).enumerate() {
    let x=i16::from_le_bytes([f[channel*2],f[channel*2+1]]) as f64/32768.0;
    if let Some(info)=rx.feed(x) {println!("ch={channel} side={side:?} t={:.4} {info:?}",n as f64/8000.0);}
   }
  }
 }
 for start in [4.709,4.729,4.749] {
  let mut m=datapump::v34::phase2::Modem::v92_digital(datapump::v90::server::ours(),8000.0);
  let mut phase=m.phase();
  for (n,f) in data.unwrap().chunks_exact(4).enumerate().skip((start*8000.0) as usize).take(8000*14) {
   let x=i16::from_le_bytes([f[0],f[1]]) as f64/32768.0;
   m.step(x);
   if m.phase()!=phase {phase=m.phase();println!("REPLAY start={start} t={:.4} phase={phase} far={:?} RTT={:?} status={:?}",n as f64/8000.0,m.far_capabilities(),m.round_trip(),m.status());}
  }
 }}