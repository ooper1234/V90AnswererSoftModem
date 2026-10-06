use ec::v42bis::{Encoder,Decoder,Params};
fn main(){
 let dir=std::env::args().nth(1).unwrap();
 let params=Params{n2:512,n7:32};
 if let Some(path)=std::env::args().nth(2){
  let wire=std::fs::read(path).unwrap();let mut d=Decoder::new(params);let mut out=Vec::new();
  d.decode(&wire,&mut out).unwrap();std::fs::write(format!("{dir}/native-decoded.bin"),out).unwrap();return;
 }
 let mut plain=Vec::new();
 for i in 0..1000u16 {plain.extend_from_slice(&[0x7e,0xff,3,0xc0,0x21,1,(i%5)as u8,0,20,2,6,0,0,0,0,5,6,0x23,0x41,0x12,0x56,0x7e]);}
 let mut seed=0x12345678u32; for round in 0..20 { for _ in 0..2048 { seed^=seed<<13; seed^=seed>>17; seed^=seed<<5; plain.push(seed as u8); } plain.extend(std::iter::repeat_n(round as u8,2048)); }
 std::fs::write(format!("{dir}/plain.bin"),&plain).unwrap();
 for chunk in [1,16,32,64,256,plain.len()]{
  let mut e=Encoder::new(params);let mut wire=Vec::new();
  for part in plain.chunks(chunk){e.encode(part,&mut wire);e.flush(&mut wire);}
  std::fs::write(format!("{dir}/ours-{chunk}.bin"),wire).unwrap();
 }
}