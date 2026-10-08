#include "hysteresis.h"
extern "C" {
void *cpp_hyst_new(bool s) { return new systemlib::Hysteresis(s); }
void cpp_hyst_free(void *h) { delete static_cast<systemlib::Hysteresis *>(h); }
bool cpp_hyst_get_state(void *h) { return static_cast<systemlib::Hysteresis *>(h)->get_state(); }
void cpp_hyst_set_time_from(void *h, bool from, uint64_t t) { static_cast<systemlib::Hysteresis *>(h)->set_hysteresis_time_from(from, t); }
void cpp_hyst_set_state_and_update(void *h, bool s, uint64_t now) { static_cast<systemlib::Hysteresis *>(h)->set_state_and_update(s, now); }
void cpp_hyst_update(void *h, uint64_t now) { static_cast<systemlib::Hysteresis *>(h)->update(now); }
}
