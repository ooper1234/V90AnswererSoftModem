use binmodemffi::*;
use datapump::framing::AsyncBits;
use std::{collections::VecDeque, ffi::{c_int,c_void,CStr}};

#[derive(Default)]
struct Dte { tx: VecDeque<bool>, rx: Vec<u8>, decoder: AsyncBitsHolder, armed: bool }
struct AsyncBitsHolder(AsyncBits);
impl Default for AsyncBitsHolder { fn default()->Self { Self(AsyncBits::new(8)) } }
extern "C" fn get(p:*mut c_void)->c_int {
    let d=unsafe{&mut *(p as *mut Dte)};
    if d.armed { d.tx.pop_front().unwrap_or(true) as c_int } else {1}
}
extern "C" fn put(p:*mut c_void,b:c_int) {
    let d=unsafe{&mut *(p as *mut Dte)};
    if d.armed { if let Some(byte)=d.decoder.0.feed(b!=0) { d.rx.push(byte); } }
}
fn run(max_rate:c_int) { run_line(max_rate,32,1,1.0,0.0) }
fn run_line(max_rate:c_int, delay:usize, echo_delay:usize, far:f64, echo:f64) {
    let sent_a:Vec<u8>=(0..1024).map(|i| if i<512 {b'A'} else {(i*73+9) as u8}).collect();
    let sent_b:Vec<u8>=(0..1024).map(|i| if i<512 {b'B'} else {(i*29+7) as u8}).collect();
    let mut a=Dte::default();let mut b=Dte::default();
    for &byte in &sent_a {a.tx.extend(AsyncBits::new(8).encode(byte));}
    for &byte in &sent_b {b.tx.extend(AsyncBits::new(8).encode(byte));}
    let ap=&mut a as *mut Dte as *mut c_void;let bp=&mut b as *mut Dte as *mut c_void;
    let end_a=bm_create_v32(1,max_rate,Some(get),ap,Some(put),ap);
    let end_b=bm_create_v32(0,max_rate,Some(get),bp,Some(put),bp);
    let mut ab=VecDeque::from(vec![0;delay]);let mut ba=ab.clone();
    let mut aa=VecDeque::from(vec![0;echo_delay]);let mut bb=aa.clone();
    for tick in 0..(80*8000) {
        if tick%160==0 {
            if !a.armed && bm_status(end_a)==BM_CONNECTED && bm_status(end_b)==BM_CONNECTED {
                bm_flush_rx(end_a);bm_flush_rx(end_b);a.armed=true;b.armed=true;
            }
            bm_service(end_a);bm_service(end_b);
        }
        let line_a=(ba.pop_front().unwrap() as f64*far+aa.pop_front().unwrap() as f64*echo).clamp(-32768.0,32767.0) as c_int;
        let x=bm_step(end_a,line_a);
        let line_b=(ab.pop_front().unwrap() as f64*far+bb.pop_front().unwrap() as f64*echo).clamp(-32768.0,32767.0) as c_int;
        let y=bm_step(end_b,line_b);
        ab.push_back(x);ba.push_back(y);aa.push_back(x);bb.push_back(y);
        for end in [end_a,end_b] {
            assert_ne!(bm_status(end),BM_FAILED,"{}",unsafe{CStr::from_ptr(bm_failure(end))}.to_string_lossy());
        }
        if a.rx.len()>=sent_b.len() && b.rx.len()>=sent_a.len() {break;}
    }
    eprintln!("rate={max_rate} status={}/{} ec={}/{} phase={}/{} received={}/{} remaining={}/{} damaged={}/{}",bm_status(end_a),bm_status(end_b),bm_error_control(end_a),bm_error_control(end_b),bm_ec_phase(end_a),bm_ec_phase(end_b),a.rx.len(),b.rx.len(),a.tx.len(),b.tx.len(),bm_damaged_frames(end_a),bm_damaged_frames(end_b));
    assert_eq!(bm_error_control(end_a),1);assert_eq!(bm_error_control(end_b),1);
    assert_eq!(bm_compression(end_a),1);assert_eq!(bm_compression(end_b),1);
    assert!(a.rx==sent_b,"answerer received {} / {} bytes",a.rx.len(),sent_b.len());
    assert!(b.rx==sent_a,"caller received {} / {} bytes",b.rx.len(),sent_a.len());
    bm_destroy(end_a);bm_destroy(end_b);
}
#[test]fn v32_with_lapm_and_compressed_bidirectional_bytes(){run(9600);}
#[test]fn v32bis_with_lapm_and_compressed_bidirectional_bytes(){run(14400);}

#[test]fn v32bis_resampled_delayed_echo_line(){run_line(14400,720,1312,0.25,0.22);}
