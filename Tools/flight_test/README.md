# Flight Test Card Data Reduction

Post-flight tooling for flight-test engineers: turn a PX4 ULog plus a
test-card CSV into a per-test-point results report right after the sortie.

```
python3 Tools/flight_test/test_card_report.py flight.ulg test_card.csv --out results/
```

The tool writes `results/report.md` (human-readable debrief) and
`results/report.csv` (one row per test point, spreadsheet-friendly) and prints
a one-line summary, e.g.

```
6 test points: 3 COMPLETE, 2 COMPLETE WITH EXCEEDANCE, 1 NO DATA -> results/report.md
```

Dependencies: Python 3.8+, `pyulog`, `numpy` (`pip install pyulog numpy`).

## Test Card Format

A CSV with exactly these columns (see `example_test_card.csv`):

| Column | Meaning |
|---|---|
| `test_point_id` | Unique ID, 1-64 characters of `A-Z a-z 0-9 _ . -` |
| `description` | Free text (max 255 characters) |
| `start_s` | Window start, seconds |
| `end_s` | Window end, seconds (must be after `start_s`) |
| `nz_max_g` | Normal load factor limit [g], may be blank |
| `airspeed_max_mps` | Calibrated airspeed limit [m/s], may be blank |
| `alt_min_m` | Altitude floor [m], may be blank |
| `alt_max_m` | Altitude ceiling [m], may be blank |

`start_s`/`end_s` are measured from the start of the log (the ULog header
timestamp, i.e. t = 0 in Flight Review) by default. Pass
`--time-origin boot` to use raw autopilot time since boot instead.

## Data Sources

| Quantity | uORB topic / field | Notes |
|---|---|---|
| Nz [g] | `vehicle_acceleration.xyz[2]` | `-z / g0`; FRD body frame, so level 1 g flight reads +1.0 |
| CAS [m/s] | `airspeed_validated.calibrated_airspeed_m_s` | Falls back to `indicated_airspeed_m_s` on logs from before the CAS field existed; NaN (invalid) samples are ignored |
| Altitude [m] | `vehicle_local_position` | `ref_alt - z` (AMSL) where `z_global` is set. If the log ever has a global reference, samples without one are excluded (noted in the report header) rather than mixed into the AMSL band; a log that never has one is reported as `-z` relative to the local origin. Samples with `z_valid == false` are ignored |
| | `vehicle_air_data.baro_alt_meter` | Fallback when `vehicle_local_position` is not logged |
| Sink rate [m/s] | `vehicle_local_position.vz` | Down positive; `v_z_valid == false` samples dropped. Falls back to `-d/dt baro_alt_meter` |
| Bank [deg] | `vehicle_attitude.q` | `abs(roll)` from the `[w, x, y, z]` quaternion |

The lowest logged instance of each topic is used (normally `multi_id 0`); if
instance 0 is missing the instance used is disclosed in the report header. When
a topic is absent from the log the affected metrics are reported as
`not logged` and any limit depending on them is listed as not evaluated. When
the topic exists but has no valid samples inside a test-point window the metric
reads `no data`.

## Per-Test-Point Results

For every test point the report contains:

- peak and mean Nz
- min and max CAS
- altitude band (min/max)
- max bank angle
- max sink rate
- duration flown: overlap between the card window and the observed data span
  (first valid sample to the end of the last valid sample's interval, i.e. its
  next logged sample; the final sample of a log owns no interval)
- every limit exceedance with the limit, the peak value, the seconds spent
  outside the limit and the time of the first exceedance. Seconds outside is
  the sum of the observed intervals of the samples beyond the limit: each
  sample owns the time up to the next logged sample (valid or invalid),
  clipped to the test-point window. Windows are sample-and-hold: the last
  sample logged before `start_s` is carried in as the value held at `start_s`,
  so an exceedance in progress when the window opens is still caught. Time
  inside invalid-data gaps and after the last sample is never counted, so the
  figure is a lower bound.

Status is one of:

- `COMPLETE` – data present in the window, no evaluated limit exceeded. A
  limit whose source topic is `not logged` / has `no data` is not evaluated;
  this is listed in the test point's notes and does not change the status
- `COMPLETE WITH EXCEEDANCE` – at least one limit exceeded
- `NO DATA` – no samples from any topic inside the window (e.g. test point
  not flown or log ended early)

`report.csv` columns: `test_point_id`, `description`, `status`, `start_s`,
`end_s`, `duration_flown_s`, `nz_peak_g`, `nz_mean_g`, `airspeed_min_mps`,
`airspeed_max_mps`, `alt_min_m`, `alt_max_m`, `bank_max_deg`,
`sink_rate_max_mps`, `exceedance_count`, `exceedances` (semicolon-separated
descriptions) and `notes`.

## Synthetic Logs

`synthetic_ulog.py` writes a ULog from a keyframe flight profile so the tool
can be exercised (and unit-tested) without a flight:

```
python3 Tools/flight_test/synthetic_ulog.py demo_flight.ulg
python3 Tools/flight_test/synthetic_ulog.py no_pitot.ulg --without airspeed_validated
python3 Tools/flight_test/test_card_report.py demo_flight.ulg Tools/flight_test/example_test_card.csv --out results/
```

The default profile is designed against `example_test_card.csv`: TP-02
overshoots its Nz limit, TP-04 busts its airspeed limit and altitude floor,
and TP-05 lies beyond the end of the log.

## Tests

```
python3 -m pytest Tools/flight_test
```
