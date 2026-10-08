//! Ports of the original HysteresisTest.cpp cases.
use px4_hysteresis::Hysteresis;
const START: u64 = 1_558_359_616_134_000;

#[test]
fn init_false() {
    assert!(!Hysteresis::new(false).get_state());
}

#[test]
fn init_true() {
    assert!(Hysteresis::new(true).get_state());
}

#[test]
fn zero() {
    let mut t = START;
    let mut h = Hysteresis::new(false);
    h.set_state_and_update(true, t);
    assert!(h.get_state());
    h.set_state_and_update(false, t);
    assert!(!h.get_state());
    h.set_state_and_update(true, t);
    assert!(h.get_state());
    t += 1000;
    h.update(t);
    assert!(h.get_state());
}

#[test]
fn change_after_time() {
    let mut t = START;
    let mut h = Hysteresis::new(false);
    h.set_hysteresis_time_from(false, 5000);
    h.set_hysteresis_time_from(true, 3000);
    h.set_state_and_update(true, t);
    assert!(!h.get_state());
    t += 4000;
    h.update(t);
    assert!(!h.get_state());
    t += 2000;
    h.update(t);
    assert!(h.get_state());
    h.set_state_and_update(false, t);
    assert!(h.get_state());
    t += 1000;
    h.update(t);
    assert!(h.get_state());
    t += 3000;
    h.update(t);
    assert!(!h.get_state());
}

#[test]
fn hysteresis_changed() {
    let mut t = START;
    let mut h = Hysteresis::new(false);
    h.set_hysteresis_time_from(true, 2000);
    h.set_hysteresis_time_from(false, 5000);
    h.set_state_and_update(true, t);
    assert!(!h.get_state());
    t += 3000;
    h.update(t);
    assert!(!h.get_state());
    t += 3000;
    h.update(t);
    assert!(h.get_state());
    h.set_hysteresis_time_from(true, 10000);
    h.set_state_and_update(false, t);
    assert!(h.get_state());
    t += 7000;
    h.update(t);
    assert!(h.get_state());
    t += 5000;
    h.update(t);
    assert!(!h.get_state());
}

#[test]
fn change_after_multiple_sets() {
    let mut t = START;
    let mut h = Hysteresis::new(false);
    h.set_hysteresis_time_from(true, 5000);
    h.set_hysteresis_time_from(false, 5000);
    h.set_state_and_update(true, t);
    assert!(!h.get_state());
    t += 3000;
    h.set_state_and_update(true, t);
    assert!(!h.get_state());
    t += 3000;
    h.set_state_and_update(true, t);
    assert!(h.get_state());
    h.set_state_and_update(false, t);
    assert!(h.get_state());
    t += 3000;
    h.set_state_and_update(false, t);
    assert!(h.get_state());
    t += 3000;
    h.set_state_and_update(false, t);
    assert!(!h.get_state());
}

#[test]
fn take_change_back() {
    let mut t = START;
    let mut h = Hysteresis::new(false);
    h.set_hysteresis_time_from(false, 5000);
    h.set_state_and_update(true, t);
    assert!(!h.get_state());
    t += 3000;
    h.update(t);
    assert!(!h.get_state());
    h.set_state_and_update(false, t);
    assert!(!h.get_state());
    t += 6000;
    h.update(t);
    assert!(!h.get_state());
    h.set_state_and_update(true, t);
    assert!(!h.get_state());
    t += 3000;
    h.update(t);
    assert!(!h.get_state());
    t += 3000;
    h.update(t);
    assert!(h.get_state());
    h.set_state_and_update(false, t);
    assert!(!h.get_state());
}
