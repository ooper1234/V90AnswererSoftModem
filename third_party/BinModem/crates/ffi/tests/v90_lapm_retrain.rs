#[path = "common/v90_lapm.rs"]
mod common;

#[test]
fn compressed_duplex_bytes_survive_two_caller_retrains() {
    common::run_full_retrain();
}

#[test]
fn compressed_fallback_bytes_survive_two_caller_retrains() {
    common::run_fallback_full_retrain();
}

#[test]
fn compressed_duplex_survives_two_caller_rate_changes() {
    common::run_caller_rate_changes();
}

#[test]
fn compressed_duplex_recovers_a_stalled_downstream() { common::run_downstream_outage(); }
