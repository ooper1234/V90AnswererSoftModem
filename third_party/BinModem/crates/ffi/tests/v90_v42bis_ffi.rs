#[path = "common/v90_lapm.rs"]
mod common;

#[test]
fn v90_v42bis_carries_async_bytes_at_daemon_cadence() {
    common::run(true, true);
}
