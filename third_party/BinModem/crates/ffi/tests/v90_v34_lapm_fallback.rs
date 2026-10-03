#[path = "common/v90_lapm.rs"]
mod common;

#[test]
fn failed_pcm_training_falls_back_to_v34_with_lapm_and_compressed_bytes() {
    common::run_fallback();
}
