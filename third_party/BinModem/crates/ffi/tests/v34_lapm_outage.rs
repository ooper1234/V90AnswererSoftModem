#[path = "common/v90_lapm.rs"]
mod common;

#[test]
fn v34_recovers_a_downstream_ack_outage_without_losing_compressed_bytes() {
    common::run_fallback_outage();
}
