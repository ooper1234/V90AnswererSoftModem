#[path = "common/v90_lapm.rs"]
mod common;

#[test]
fn v90_compression_can_be_disabled_without_disabling_lapm() {
    common::run(true, false);
}
