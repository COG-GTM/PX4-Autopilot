# Flight test card data reduction

Post-flight tooling that turns a ULog plus a test-card definition into a
per-test-point results report, so achieved conditions and limit exceedances
can be reviewed right after the sortie.

Requirements: Python 3, `numpy` and `pyulog` (both in
`Tools/setup/requirements.txt`).

## Usage

```sh
python3 Tools/flight_test/test_card_report.py flight.ulg test_card.csv --out results/
```

Writes `results/report.md` (sortie summary table plus a detail section per
test point) and `results/report.csv` (one machine-readable row per test
point), and prints a one-line summary:

```
6 test points: 4 complete, 1 with exceedance, 1 no data -> results/report.md, results/report.csv
```

Each test point is marked:

| Status | Meaning |
|---|---|
| `COMPLETE` | Data present in the window, every limit on the card evaluated, none exceeded |
| `COMPLETE WITH EXCEEDANCE` | Data present, at least one limit exceeded (details listed) |
| `NO DATA` | No samples in the window, or a limit on the card could not be evaluated because its channel is not logged |

## Test card CSV schema

One row per test point. Times are seconds from log start; limit columns may be
left blank to skip that check. See `example_test_card.csv`.

| Column | Required | Meaning |
|---|---|---|
| `test_point_id` | yes | Unique identifier, `[A-Za-z0-9._-]`, up to 64 characters |
| `description` | yes | Free text (up to 255 characters, may be empty) |
| `start_s` | yes | Window start, seconds from log start (>= 0) |
| `end_s` | yes | Window end, seconds from log start (> `start_s`) |
| `nz_max_g` | no | Maximum allowed normal load factor [g] |
| `airspeed_max_mps` | no | Maximum allowed calibrated airspeed [m/s] |
| `alt_min_m` | no | Minimum allowed altitude [m] |
| `alt_max_m` | no | Maximum allowed altitude [m] (must exceed `alt_min_m`) |

Malformed cards (bad identifiers, non-numeric values, inverted windows or
altitude bands, duplicate identifiers, unknown columns) are rejected with an
error rather than silently skipped.

## Log channels

| Channel | Source topic / field | Fallback |
|---|---|---|
| Nz | `vehicle_acceleration.xyz[2]`, FRD body z specific force, `Nz = -az / g0` | - |
| Calibrated airspeed | `airspeed_validated.calibrated_airspeed_m_s` (NaN samples dropped) | - |
| Altitude | `vehicle_local_position.ref_alt - z` when `z_global` is set | `vehicle_air_data.baro_alt_meter`; `-z` relative to origin if no global reference |
| Sink rate | `vehicle_local_position.vz` (down positive) | `-d/dt vehicle_air_data.baro_alt_meter` |
| Bank angle | roll from `vehicle_attitude.q` | - |

A topic that is absent from the log is reported as "not logged" in the data
sources table, in the per-test-point notes, and in the `notes` column of the
CSV; the affected statistics are left as `n/a` (Markdown) or empty (CSV).

Per test point the tool reports: peak and mean Nz, min/max calibrated
airspeed, altitude band flown, maximum absolute bank angle, maximum sink rate,
duration actually flown (span of logged samples inside the window), and for
each exceeded limit the peak value, the seconds spent outside the limit and
the time of the first exceedance.

## report.csv columns

| Column | Meaning |
|---|---|
| `test_point_id` | From the test card |
| `description` | From the test card |
| `start_s` | Window start [s] |
| `end_s` | Window end [s] |
| `status` | `COMPLETE`, `COMPLETE WITH EXCEEDANCE` or `NO DATA` |
| `duration_flown_s` | Span of logged samples inside the window [s] |
| `nz_peak_g` | Peak Nz [g] |
| `nz_mean_g` | Mean Nz [g] |
| `airspeed_min_mps` | Minimum calibrated airspeed [m/s] |
| `airspeed_max_mps` | Maximum calibrated airspeed [m/s] |
| `alt_min_flown_m` | Lowest altitude flown [m] |
| `alt_max_flown_m` | Highest altitude flown [m] |
| `bank_max_deg` | Maximum absolute bank angle [deg] |
| `sink_rate_max_mps` | Maximum sink rate [m/s] |
| `exceedance_count` | Number of exceeded limits |
| `exceedances` | `;`-separated `limit=value peak=... outside=...s first=...s` entries |
| `notes` | `;`-separated data-availability notes |

## Synthetic data

`make_synthetic_ulog.py` writes a ULog with the topics above for a scripted
profile (trim, climb, level acceleration, a wind-up turn to 70 deg of bank that
busts a 2.5 g Nz limit, descent) so the tool can be exercised without flight
data:

```sh
python3 Tools/flight_test/make_synthetic_ulog.py --out synthetic.ulg
python3 Tools/flight_test/test_card_report.py synthetic.ulg Tools/flight_test/example_test_card.csv --out results/
```

`--skip-topic <name>` omits a topic to exercise the "not logged" paths. The
generator writes the ULog binary format directly (pyulog only serialises logs
it has parsed) and the result is read back with pyulog in the tests.

## Tests

```sh
python3 -m pytest Tools/flight_test
```
