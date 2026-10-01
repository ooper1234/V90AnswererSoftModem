//! Default V.90 against a peer that offers no LAPM: retain the raw data path.
#[path = "common/v90_raw.rs"]
mod common;

#[test]
fn v90_without_peer_lapm_falls_back_to_transparent_data() {
    unsafe { std::env::remove_var("V90_ERROR_CONTROL"); }
    common::run();
}
