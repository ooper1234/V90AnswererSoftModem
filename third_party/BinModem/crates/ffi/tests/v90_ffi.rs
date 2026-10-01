#[path = "common/v90_raw.rs"]
mod common;

#[test]
fn the_ffi_v90_stage_connects_downstream_and_carries_bits() {
    unsafe { std::env::set_var("V90_ERROR_CONTROL", "0"); }
    common::run();
}
