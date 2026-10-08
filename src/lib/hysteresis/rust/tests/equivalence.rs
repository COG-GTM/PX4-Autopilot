//! Runs identical random input sequences through the original C++ and the Rust port
//! and requires identical output after every step.
use px4_hysteresis::Hysteresis;
use std::ffi::c_void;

extern "C" {
    fn cpp_hyst_new(s: bool) -> *mut c_void;
    fn cpp_hyst_free(h: *mut c_void);
    fn cpp_hyst_get_state(h: *mut c_void) -> bool;
    fn cpp_hyst_set_time_from(h: *mut c_void, from: bool, t: u64);
    fn cpp_hyst_set_state_and_update(h: *mut c_void, s: bool, now: u64);
    fn cpp_hyst_update(h: *mut c_void, now: u64);
}

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 33
    }
}

#[test]
fn rust_matches_cpp_on_random_sequences() {
    let mut rng = Lcg(42);
    let mut steps = 0u64;
    for _ in 0..10_000 {
        let init = rng.next() & 1 == 0;
        let mut rs = Hysteresis::new(init);
        let cpp = unsafe { cpp_hyst_new(init) };
        let mut now: u64 = 1_558_359_616_134_000;
        for _ in 0..200 {
            match rng.next() % 4 {
                0 => {
                    let (from, t) = (rng.next() & 1 == 0, rng.next() % 10_000);
                    rs.set_hysteresis_time_from(from, t);
                    unsafe { cpp_hyst_set_time_from(cpp, from, t) };
                }
                1 | 2 => {
                    let s = rng.next() & 1 == 0;
                    rs.set_state_and_update(s, now);
                    unsafe { cpp_hyst_set_state_and_update(cpp, s, now) };
                }
                _ => {
                    rs.update(now);
                    unsafe { cpp_hyst_update(cpp, now) };
                }
            }
            assert_eq!(
                rs.get_state(),
                unsafe { cpp_hyst_get_state(cpp) },
                "diverged at step {steps}"
            );
            now += rng.next() % 3_000;
            steps += 1;
        }
        unsafe { cpp_hyst_free(cpp) };
    }
    println!("{steps} steps compared, 0 differences");
}
