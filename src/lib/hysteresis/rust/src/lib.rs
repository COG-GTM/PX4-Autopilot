//! Rust port of PX4 `systemlib::Hysteresis` (src/lib/hysteresis).

pub type HrtAbstime = u64;

#[derive(Debug, Default, Clone)]
pub struct Hysteresis {
    last_time_to_change_state: HrtAbstime,
    time_from_true_us: HrtAbstime,
    time_from_false_us: HrtAbstime,
    state: bool,
    requested_state: bool,
}

impl Hysteresis {
    pub fn new(initial_state: bool) -> Self {
        Self {
            state: initial_state,
            requested_state: initial_state,
            ..Default::default()
        }
    }

    pub fn get_state(&self) -> bool {
        self.state
    }

    pub fn set_hysteresis_time_from(
        &mut self,
        from_state: bool,
        new_hysteresis_time_us: HrtAbstime,
    ) {
        if from_state {
            self.time_from_true_us = new_hysteresis_time_us;
        } else {
            self.time_from_false_us = new_hysteresis_time_us;
        }
    }

    pub fn set_state_and_update(&mut self, new_state: bool, now_us: HrtAbstime) {
        if new_state != self.state {
            if new_state != self.requested_state {
                self.requested_state = new_state;
                self.last_time_to_change_state = now_us;
            }
        } else {
            self.requested_state = self.state;
        }
        self.update(now_us);
    }

    pub fn update(&mut self, now_us: HrtAbstime) {
        if self.requested_state == self.state {
            return;
        }
        // C++ uses unsigned (wrapping) addition here.
        let hold = if self.state {
            self.time_from_true_us
        } else {
            self.time_from_false_us
        };
        if now_us >= self.last_time_to_change_state.wrapping_add(hold) {
            self.state = self.requested_state;
        }
    }
}
