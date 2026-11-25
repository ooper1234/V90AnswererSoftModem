//! Where one sample's step spent its time, for the host's slow-step log.
//! Each section adds its wall time to a per-thread list; the host clears the
//! list before a step and reads it after one that ran long.
use std::cell::RefCell;

thread_local! {
    static SECTIONS: RefCell<Vec<(&'static str, f64)>> = const { RefCell::new(Vec::new()) };
}

pub fn add(name: &'static str, ms: f64) {
    SECTIONS.with(|s| {
        let mut s = s.borrow_mut();
        match s.iter_mut().find(|(n, _)| *n == name) {
            Some(slot) => slot.1 += ms,
            None => s.push((name, ms)),
        }
    });
}

pub fn take() -> Vec<(&'static str, f64)> {
    SECTIONS.with(|s| std::mem::take(&mut *s.borrow_mut()))
}

/// Time `$e` as section `$name`.
#[macro_export]
macro_rules! dprof {
    ($name:expr, $e:expr) => {{
        let t = std::time::Instant::now();
        let r = $e;
        $crate::prof::add($name, t.elapsed().as_secs_f64() * 1000.0);
        r
    }};
}
