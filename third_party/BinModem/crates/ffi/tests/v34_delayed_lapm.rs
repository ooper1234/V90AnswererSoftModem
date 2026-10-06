#[path = "common/v90_lapm.rs"]
mod common;

#[test]
fn v34_fallback_transfers_compressed_duplex_bytes_with_300_ms_added_rtt() {
    // This integration test has one test in its own process. The ordinary
    // fixture adds 15 ms each way; add the hardware experiment's 150 ms.
    unsafe { std::env::set_var("LAPM_TEST_DELAY_MS", "165"); }
    common::run_fallback();
}
