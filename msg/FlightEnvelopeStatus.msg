# Flight-test envelope monitor status
#
# Advisory comparison of the current flight state against the cleared test
# envelope. Published by the flight_envelope_monitor module at a bounded rate.

uint64 timestamp                   # time since system start (microseconds)
uint64 timestamp_sample            # timestamp of the newest input sample used for this evaluation (microseconds)

bool monitoring_active             # true while the monitor is evaluating (armed and airborne)

float32 load_factor                # [g] current normal load factor Nz (+1 g in level flight), NaN if invalid
float32 airspeed                   # [m/s] current calibrated airspeed, NaN if invalid
float32 altitude                   # [m] current altitude AMSL, NaN if invalid
float32 vertical_speed             # [m/s] current vertical speed (positive up), NaN if invalid
float32 angle_of_attack            # [deg] current angle of attack, NaN if no source is available

uint8 STATE_NOMINAL = 0
uint8 STATE_WARNING = 1
uint8 STATE_CRITICAL = 2

uint8 CHANNEL_LOAD_FACTOR_HIGH = 0
uint8 CHANNEL_LOAD_FACTOR_LOW = 1
uint8 CHANNEL_AIRSPEED = 2
uint8 CHANNEL_ALTITUDE_HIGH = 3
uint8 CHANNEL_ALTITUDE_LOW = 4
uint8 CHANNEL_SINK_RATE = 5
uint8 CHANNEL_ANGLE_OF_ATTACK = 6
uint8 CHANNEL_COUNT = 7

uint8[7] channel_state             # per-channel state, indexed by CHANNEL_*, values STATE_*
uint8 exceedance_mask              # bit set for each channel currently in WARNING or CRITICAL, bit index = CHANNEL_*
uint8 worst_state                  # worst state over all channels, values STATE_*

float32 peak_load_factor_max       # [g] highest load factor seen so far this flight
float32 peak_load_factor_min       # [g] lowest load factor seen so far this flight

float32 time_in_warning_s          # [s] cumulative time this flight with worst_state == WARNING
float32 time_in_critical_s         # [s] cumulative time this flight with worst_state == CRITICAL
