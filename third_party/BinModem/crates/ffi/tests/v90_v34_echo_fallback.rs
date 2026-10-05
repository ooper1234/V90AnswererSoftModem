#[path = "common/v90_lapm.rs"]
mod common;

#[test]
fn v90_fallback_carries_compressed_bytes_with_delayed_echo() {
    common::run_fallback_echo();
}
