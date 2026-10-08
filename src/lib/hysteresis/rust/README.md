# Hysteresis: Rust port

Rust version of `systemlib::Hysteresis` (`../hysteresis.cpp`). Hysteresis is a
debounce: a requested on/off change only takes effect after it has held for a set
time. PX4 uses it in land detection, failure detection and arming checks.

## How equivalence is checked

- `tests/ported_gtests.rs`: every case from `../HysteresisTest.cpp`, ported 1:1.
- `tests/equivalence.rs`: `build.rs` compiles the original C++ next to the Rust
  port. The test drives both with the same 2,000,000 random calls and fails on the
  first output that differs.

```
cargo test
```

Not wired into the PX4 firmware build. Integration into the flight stack is a
separate step.
