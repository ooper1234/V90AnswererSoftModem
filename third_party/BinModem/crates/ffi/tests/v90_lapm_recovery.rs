#[path = "common/v90_lapm.rs"]
mod common;

#[test]
fn compressed_duplex_bytes_survive_two_upstream_playout_jumps() {
    common::run_receive_recovery();
}

#[test]
fn compressed_duplex_survives_whole_carrier_cycle_frame_clock_jump() {
    common::run_whole_jump();
}
