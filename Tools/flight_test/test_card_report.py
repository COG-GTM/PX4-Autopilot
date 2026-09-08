#!/usr/bin/env python3
"""
Post-flight test-card data reduction for PX4 ULog files.

Reads a flight log and a test-card CSV (one row per test point with a time
window and optional limits) and writes a per-test-point results report:

    python3 Tools/flight_test/test_card_report.py flight.ulg test_card.csv --out results/

Outputs ``report.md`` (summary table plus per-test-point detail) and
``report.csv`` (one row per test point) in the output directory and prints a
one-line summary to stdout.

Test card CSV columns (times are seconds from log start; limits may be blank):

    test_point_id, description, start_s, end_s,
    nz_max_g, airspeed_max_mps, alt_min_m, alt_max_m

Data sources (each falls back to a "not logged" note when absent):

    Nz              vehicle_acceleration.xyz[2]      (FRD body z specific force)
    airspeed        airspeed_validated.calibrated_airspeed_m_s
    altitude        vehicle_local_position.ref_alt - z  (when z_global) else
                    vehicle_air_data.baro_alt_meter
    sink rate       vehicle_local_position.vz (down positive) else
                    -d/dt of smoothed vehicle_air_data.baro_alt_meter
    bank angle      vehicle_attitude.q (roll from quaternion)
"""

import argparse
import csv
import math
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from pyulog import ULog

STANDARD_GRAVITY_MPS2 = 9.80665
# Moving-average length applied to barometric altitude before differentiating
# it into a fallback sink rate; raw baro noise would otherwise dominate d/dt.
BARO_SINK_RATE_SMOOTHING_S = 1.0

STATUS_COMPLETE = "COMPLETE"
STATUS_EXCEEDANCE = "COMPLETE WITH EXCEEDANCE"
STATUS_NO_DATA = "NO DATA"

TEST_POINT_ID_PATTERN = re.compile(r"^[A-Za-z0-9._-]{1,64}$")
MAX_DESCRIPTION_CHARS = 255

REQUIRED_TEST_CARD_COLUMNS = ("test_point_id", "description", "start_s", "end_s")
LIMIT_COLUMNS = ("nz_max_g", "airspeed_max_mps", "alt_min_m", "alt_max_m")
TEST_CARD_COLUMNS = REQUIRED_TEST_CARD_COLUMNS + LIMIT_COLUMNS

# limit column -> (channel key, "max" or "min" bound)
LIMIT_CHANNELS = {
    "nz_max_g": ("nz_g", "max"),
    "airspeed_max_mps": ("airspeed_mps", "max"),
    "alt_min_m": ("altitude_m", "min"),
    "alt_max_m": ("altitude_m", "max"),
}

CHANNEL_LABELS = {
    "nz_g": "Nz [g]",
    "airspeed_mps": "CAS [m/s]",
    "altitude_m": "Altitude [m]",
    "sink_rate_mps": "Sink rate [m/s]",
    "bank_deg": "Bank [deg]",
}

REPORT_CSV_COLUMNS = (
    "test_point_id",
    "description",
    "start_s",
    "end_s",
    "status",
    "duration_flown_s",
    "nz_peak_g",
    "nz_mean_g",
    "airspeed_min_mps",
    "airspeed_max_mps",
    "alt_min_flown_m",
    "alt_max_flown_m",
    "bank_max_deg",
    "sink_rate_max_mps",
    "exceedance_count",
    "exceedances",
    "notes",
)


class TestCardError(ValueError):
    """Raised for a malformed test card."""


@dataclass(frozen=True)
class TestPoint:
    test_point_id: str
    description: str
    start_s: float
    end_s: float
    nz_max_g: Optional[float] = None
    airspeed_max_mps: Optional[float] = None
    alt_min_m: Optional[float] = None
    alt_max_m: Optional[float] = None

    def limits(self) -> Dict[str, float]:
        """Populated limit columns as {column: value}."""
        return {name: getattr(self, name) for name in LIMIT_COLUMNS if getattr(self, name) is not None}


@dataclass(frozen=True)
class Channel:
    """A time series (seconds from log start) with its provenance."""

    key: str
    source: str
    t: np.ndarray
    values: np.ndarray

    def window(self, start_s: float, end_s: float) -> "Channel":
        mask = (self.t >= start_s) & (self.t <= end_s)
        return Channel(self.key, self.source, self.t[mask], self.values[mask])

    def __len__(self) -> int:
        return len(self.t)


@dataclass(frozen=True)
class Exceedance:
    limit: str
    limit_value: float
    peak_value: float
    seconds_outside: float
    first_exceedance_s: float


@dataclass
class TestPointResult:
    test_point: TestPoint
    status: str = STATUS_NO_DATA
    duration_flown_s: Optional[float] = None
    nz_peak_g: Optional[float] = None
    nz_mean_g: Optional[float] = None
    airspeed_min_mps: Optional[float] = None
    airspeed_max_mps: Optional[float] = None
    alt_min_flown_m: Optional[float] = None
    alt_max_flown_m: Optional[float] = None
    bank_max_deg: Optional[float] = None
    sink_rate_max_mps: Optional[float] = None
    exceedances: List[Exceedance] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


# --------------------------------------------------------------------------- test card


def _parse_optional_float(raw: str, column: str, row_number: int) -> Optional[float]:
    text = raw.strip()
    if text == "":
        return None
    try:
        value = float(text)
    except ValueError as exc:
        raise TestCardError(f"row {row_number}: {column} must be numeric, got {text!r}") from exc
    if not math.isfinite(value):
        raise TestCardError(f"row {row_number}: {column} must be finite, got {text!r}")
    return value


def parse_test_card_rows(rows: Sequence[Dict[str, str]]) -> List[TestPoint]:
    """Validate and convert CSV dict rows into TestPoint objects."""
    test_points: List[TestPoint] = []
    seen_ids = set()
    for index, row in enumerate(rows):
        row_number = index + 2  # 1-based, after the header line
        test_point_id = (row.get("test_point_id") or "").strip()
        if not TEST_POINT_ID_PATTERN.match(test_point_id):
            raise TestCardError(
                f"row {row_number}: test_point_id {test_point_id!r} must match {TEST_POINT_ID_PATTERN.pattern}"
            )
        if test_point_id in seen_ids:
            raise TestCardError(f"row {row_number}: duplicate test_point_id {test_point_id!r}")
        seen_ids.add(test_point_id)

        description = (row.get("description") or "").strip()
        if len(description) > MAX_DESCRIPTION_CHARS:
            raise TestCardError(f"row {row_number}: description longer than {MAX_DESCRIPTION_CHARS} characters")

        start_s = _parse_optional_float(row.get("start_s") or "", "start_s", row_number)
        end_s = _parse_optional_float(row.get("end_s") or "", "end_s", row_number)
        if start_s is None or end_s is None:
            raise TestCardError(f"row {row_number}: start_s and end_s are required")
        if start_s < 0 or end_s <= start_s:
            raise TestCardError(f"row {row_number}: need 0 <= start_s < end_s, got {start_s} .. {end_s}")

        limits = {name: _parse_optional_float(row.get(name) or "", name, row_number) for name in LIMIT_COLUMNS}
        if limits["nz_max_g"] is not None and limits["nz_max_g"] <= 0:
            raise TestCardError(f"row {row_number}: nz_max_g must be positive")
        if limits["airspeed_max_mps"] is not None and limits["airspeed_max_mps"] <= 0:
            raise TestCardError(f"row {row_number}: airspeed_max_mps must be positive")
        if (
            limits["alt_min_m"] is not None
            and limits["alt_max_m"] is not None
            and limits["alt_min_m"] >= limits["alt_max_m"]
        ):
            raise TestCardError(f"row {row_number}: alt_min_m must be below alt_max_m")

        test_points.append(TestPoint(test_point_id, description, start_s, end_s, **limits))

    if not test_points:
        raise TestCardError("test card has no test points")
    return test_points


def read_test_card(path: str) -> List[TestPoint]:
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        header = [name.strip() for name in (reader.fieldnames or [])]
        missing = [name for name in REQUIRED_TEST_CARD_COLUMNS if name not in header]
        if missing:
            raise TestCardError(f"test card is missing required column(s): {', '.join(missing)}")
        unknown = [name for name in header if name not in TEST_CARD_COLUMNS]
        if unknown:
            raise TestCardError(f"test card has unknown column(s): {', '.join(unknown)}")
        rows = [{(k or "").strip(): (v or "") for k, v in row.items()} for row in reader]
    return parse_test_card_rows(rows)


# --------------------------------------------------------------------------- log channels


def _moving_average(values: np.ndarray, t: np.ndarray, window_s: float) -> np.ndarray:
    """Centred moving average over roughly ``window_s`` of samples (edge-normalised)."""
    dt = np.median(np.diff(t))
    if not np.isfinite(dt) or dt <= 0:
        return values
    length = max(1, int(round(window_s / dt)))
    if length <= 1:
        return values
    kernel = np.ones(length)
    return np.convolve(values, kernel, mode="same") / np.convolve(np.ones_like(values), kernel, mode="same")


class FlightLog:
    """Channels extracted from a ULog, all on a common seconds-from-log-start time base."""

    def __init__(self, ulog: ULog):
        self._ulog = ulog
        self.start_timestamp_us = self._resolve_start_timestamp()
        self.channels: Dict[str, Optional[Channel]] = {}
        self.source_notes: Dict[str, str] = {}
        self._extract()

    @classmethod
    def from_file(cls, path: str) -> "FlightLog":
        return cls(ULog(path, message_name_filter_list=[
            "vehicle_acceleration",
            "airspeed_validated",
            "vehicle_local_position",
            "vehicle_air_data",
            "vehicle_attitude",
        ]))

    @property
    def duration_s(self) -> float:
        return (self._ulog.last_timestamp - self.start_timestamp_us) / 1e6

    def _resolve_start_timestamp(self) -> int:
        if self._ulog.start_timestamp > 0:
            return self._ulog.start_timestamp
        first = [int(d.data["timestamp"][0]) for d in self._ulog.data_list if len(d.data["timestamp"])]
        return min(first) if first else 0

    def _dataset(self, name: str):
        try:
            return self._ulog.get_dataset(name)
        except (KeyError, IndexError, ValueError):
            return None

    def _seconds(self, dataset) -> np.ndarray:
        return (dataset.data["timestamp"].astype(np.int64) - self.start_timestamp_us) / 1e6

    def _make_channel(self, key: str, source: str, t: np.ndarray, values: np.ndarray) -> Optional[Channel]:
        values = np.asarray(values, dtype=np.float64)
        valid = np.isfinite(values)
        if not valid.any():
            self.source_notes[key] = f"{source}: logged but no finite samples"
            return None
        self.source_notes[key] = source
        return Channel(key, source, t[valid], values[valid])

    def _extract(self) -> None:
        self.channels["nz_g"] = self._extract_nz()
        self.channels["airspeed_mps"] = self._extract_airspeed()
        self.channels["altitude_m"], self.channels["sink_rate_mps"] = self._extract_altitude_and_sink()
        self.channels["bank_deg"] = self._extract_bank()

    def _extract_nz(self) -> Optional[Channel]:
        accel = self._dataset("vehicle_acceleration")
        if accel is None or "xyz[2]" not in accel.data:
            self.source_notes["nz_g"] = "vehicle_acceleration not logged"
            return None
        nz = -accel.data["xyz[2]"] / STANDARD_GRAVITY_MPS2
        return self._make_channel("nz_g", "vehicle_acceleration.xyz[2]", self._seconds(accel), nz)

    def _extract_airspeed(self) -> Optional[Channel]:
        airspeed = self._dataset("airspeed_validated")
        if airspeed is None or "calibrated_airspeed_m_s" not in airspeed.data:
            self.source_notes["airspeed_mps"] = "airspeed_validated not logged"
            return None
        return self._make_channel(
            "airspeed_mps",
            "airspeed_validated.calibrated_airspeed_m_s",
            self._seconds(airspeed),
            airspeed.data["calibrated_airspeed_m_s"],
        )

    def _extract_altitude_and_sink(self) -> Tuple[Optional[Channel], Optional[Channel]]:
        altitude: Optional[Channel] = None
        sink: Optional[Channel] = None

        local_position = self._dataset("vehicle_local_position")
        if local_position is not None and {"z", "z_global", "ref_alt"} <= set(local_position.data):
            t = self._seconds(local_position)
            z = local_position.data["z"].astype(np.float64)
            z_global = local_position.data["z_global"].astype(bool)
            if z_global.all():
                altitude = self._make_channel(
                    "altitude_m", "vehicle_local_position.ref_alt - z", t, local_position.data["ref_alt"] - z
                )
            elif not z_global.any():
                altitude = self._make_channel(
                    "altitude_m", "vehicle_local_position (-z, relative to origin: no global reference)", t, -z
                )
            if "vz" in local_position.data:
                sink = self._make_channel("sink_rate_mps", "vehicle_local_position.vz", t, local_position.data["vz"])

        air_data = self._dataset("vehicle_air_data")
        if air_data is not None and "baro_alt_meter" in air_data.data:
            t = self._seconds(air_data)
            baro_alt = air_data.data["baro_alt_meter"].astype(np.float64)
            if altitude is None:
                altitude = self._make_channel("altitude_m", "vehicle_air_data.baro_alt_meter", t, baro_alt)
            if sink is None and len(t) > 1:
                sink = self._make_channel(
                    "sink_rate_mps",
                    f"-d/dt vehicle_air_data.baro_alt_meter ({BARO_SINK_RATE_SMOOTHING_S:g} s moving average)",
                    t,
                    -np.gradient(_moving_average(baro_alt, t, BARO_SINK_RATE_SMOOTHING_S), t),
                )

        if altitude is None:
            self.source_notes.setdefault(
                "altitude_m", "vehicle_local_position (with global reference) and vehicle_air_data not logged"
            )
        if sink is None:
            self.source_notes.setdefault("sink_rate_mps", "vehicle_local_position and vehicle_air_data not logged")
        return altitude, sink

    def _extract_bank(self) -> Optional[Channel]:
        attitude = self._dataset("vehicle_attitude")
        if attitude is None or not {"q[0]", "q[1]", "q[2]", "q[3]"} <= set(attitude.data):
            self.source_notes["bank_deg"] = "vehicle_attitude not logged"
            return None
        q0, q1, q2, q3 = (attitude.data[f"q[{i}]"].astype(np.float64) for i in range(4))
        roll = np.arctan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
        return self._make_channel("bank_deg", "vehicle_attitude.q (roll)", self._seconds(attitude), np.degrees(roll))


# --------------------------------------------------------------------------- reduction


def _sample_durations(t: np.ndarray) -> np.ndarray:
    """Per-sample hold time; the last sample gets the median spacing."""
    if len(t) == 1:
        return np.zeros(1)
    dt = np.diff(t)
    return np.append(dt, np.median(dt))


def find_exceedance(channel: Channel, limit: str, limit_value: float, bound: str) -> Optional[Exceedance]:
    outside = channel.values > limit_value if bound == "max" else channel.values < limit_value
    if not outside.any():
        return None
    peak = float(channel.values[outside].max() if bound == "max" else channel.values[outside].min())
    seconds_outside = float(_sample_durations(channel.t)[outside].sum())
    first = float(channel.t[np.argmax(outside)])
    return Exceedance(limit, limit_value, peak, seconds_outside, first)


def reduce_test_point(test_point: TestPoint, log: FlightLog) -> TestPointResult:
    result = TestPointResult(test_point)
    windowed: Dict[str, Optional[Channel]] = {}
    for key, channel in log.channels.items():
        if channel is None:
            windowed[key] = None
            result.notes.append(f"{CHANNEL_LABELS[key]}: {log.source_notes[key]}")
            continue
        chan = channel.window(test_point.start_s, test_point.end_s)
        if len(chan) == 0:
            windowed[key] = None
            result.notes.append(f"{CHANNEL_LABELS[key]}: no samples in window")
        else:
            windowed[key] = chan

    present = [c for c in windowed.values() if c is not None]
    if present:
        first = min(float(c.t[0]) for c in present)
        last = max(float(c.t[-1]) for c in present)
        result.duration_flown_s = last - first

    nz, airspeed, altitude = windowed["nz_g"], windowed["airspeed_mps"], windowed["altitude_m"]
    sink, bank = windowed["sink_rate_mps"], windowed["bank_deg"]
    if nz is not None:
        result.nz_peak_g = float(nz.values.max())
        result.nz_mean_g = float(nz.values.mean())
    if airspeed is not None:
        result.airspeed_min_mps = float(airspeed.values.min())
        result.airspeed_max_mps = float(airspeed.values.max())
    if altitude is not None:
        result.alt_min_flown_m = float(altitude.values.min())
        result.alt_max_flown_m = float(altitude.values.max())
    if bank is not None:
        result.bank_max_deg = float(np.abs(bank.values).max())
    if sink is not None:
        result.sink_rate_max_mps = float(sink.values.max())

    limit_not_evaluated = False
    for limit, limit_value in test_point.limits().items():
        channel_key, bound = LIMIT_CHANNELS[limit]
        channel = windowed[channel_key]
        if channel is None:
            limit_not_evaluated = True
            result.notes.append(f"limit {limit}={limit_value:g} not evaluated: no {CHANNEL_LABELS[channel_key]} data")
            continue
        exceedance = find_exceedance(channel, limit, limit_value, bound)
        if exceedance is not None:
            result.exceedances.append(exceedance)

    if not present or limit_not_evaluated:
        result.status = STATUS_NO_DATA
    elif result.exceedances:
        result.status = STATUS_EXCEEDANCE
    else:
        result.status = STATUS_COMPLETE
    return result


def reduce_test_card(test_points: Sequence[TestPoint], log: FlightLog) -> List[TestPointResult]:
    return [reduce_test_point(tp, log) for tp in test_points]


# --------------------------------------------------------------------------- output


def _fmt(value: Optional[float], digits: int = 2) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def _md_escape(text: str) -> str:
    return text.replace("|", "\\|").replace("\n", " ")


def _exceedance_summary(exceedance: Exceedance) -> str:
    return (
        f"{exceedance.limit}={exceedance.limit_value:g} peak={exceedance.peak_value:.2f} "
        f"outside={exceedance.seconds_outside:.2f}s first={exceedance.first_exceedance_s:.2f}s"
    )


def summary_line(results: Sequence[TestPointResult], out_dir: str) -> str:
    counts = {status: 0 for status in (STATUS_COMPLETE, STATUS_EXCEEDANCE, STATUS_NO_DATA)}
    for result in results:
        counts[result.status] += 1
    return (
        f"{len(results)} test points: {counts[STATUS_COMPLETE]} complete, "
        f"{counts[STATUS_EXCEEDANCE]} with exceedance, {counts[STATUS_NO_DATA]} no data "
        f"-> {os.path.join(out_dir, 'report.md')}, {os.path.join(out_dir, 'report.csv')}"
    )


def render_markdown(results: Sequence[TestPointResult], log: FlightLog, log_path: str, card_path: str) -> str:
    lines = [
        "# Flight test card report",
        "",
        f"- Log: `{os.path.basename(log_path)}` ({log.duration_s:.1f} s)",
        f"- Test card: `{os.path.basename(card_path)}`",
        "- Times are seconds from log start.",
        "",
        "## Data sources",
        "",
        "| Channel | Source |",
        "|---|---|",
    ]
    for key in CHANNEL_LABELS:
        lines.append(f"| {CHANNEL_LABELS[key]} | {log.source_notes.get(key, 'not logged')} |")

    lines += [
        "",
        "## Sortie summary",
        "",
        "| Test point | Description | Window [s] | Flown [s] | Nz peak [g] | Nz mean [g] | CAS min/max [m/s] "
        "| Alt band [m] | Bank max [deg] | Sink max [m/s] | Exceedances | Status |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        tp = r.test_point
        lines.append(
            f"| {_md_escape(tp.test_point_id)} | {_md_escape(tp.description)} | {tp.start_s:g}-{tp.end_s:g} "
            f"| {_fmt(r.duration_flown_s, 1)} | {_fmt(r.nz_peak_g)} | {_fmt(r.nz_mean_g)} "
            f"| {_fmt(r.airspeed_min_mps, 1)} / {_fmt(r.airspeed_max_mps, 1)} "
            f"| {_fmt(r.alt_min_flown_m, 0)} - {_fmt(r.alt_max_flown_m, 0)} | {_fmt(r.bank_max_deg, 1)} "
            f"| {_fmt(r.sink_rate_max_mps, 1)} | {len(r.exceedances)} | **{r.status}** |"
        )

    lines += ["", "## Test point details"]
    for r in results:
        tp = r.test_point
        lines += [
            "",
            f"### {_md_escape(tp.test_point_id)} - {_md_escape(tp.description)}",
            "",
            f"- Status: **{r.status}**",
            f"- Window: {tp.start_s:g} s to {tp.end_s:g} s (planned {tp.end_s - tp.start_s:g} s, "
            f"flown {_fmt(r.duration_flown_s, 1)} s)",
        ]
        limits = tp.limits()
        lines.append(
            "- Limits: " + (", ".join(f"{name}={value:g}" for name, value in limits.items()) if limits else "none")
        )
        lines += [
            f"- Nz: peak {_fmt(r.nz_peak_g)} g, mean {_fmt(r.nz_mean_g)} g",
            f"- CAS: min {_fmt(r.airspeed_min_mps, 1)} m/s, max {_fmt(r.airspeed_max_mps, 1)} m/s",
            f"- Altitude band: {_fmt(r.alt_min_flown_m, 1)} m to {_fmt(r.alt_max_flown_m, 1)} m",
            f"- Bank angle: max {_fmt(r.bank_max_deg, 1)} deg",
            f"- Sink rate: max {_fmt(r.sink_rate_max_mps, 1)} m/s",
        ]
        if r.exceedances:
            lines += [
                "",
                "| Limit | Limit value | Peak | Seconds outside | First exceedance [s] |",
                "|---|---|---|---|---|",
            ]
            for e in r.exceedances:
                lines.append(
                    f"| {e.limit} | {e.limit_value:g} | {e.peak_value:.2f} | {e.seconds_outside:.2f} "
                    f"| {e.first_exceedance_s:.2f} |"
                )
        else:
            lines.append("- Exceedances: none")
        if r.notes:
            lines.append("- Notes:")
            lines.extend(f"  - {_md_escape(note)}" for note in r.notes)
    lines.append("")
    return "\n".join(lines)


def result_to_csv_row(result: TestPointResult) -> Dict[str, str]:
    tp = result.test_point

    def num(value: Optional[float]) -> str:
        return "" if value is None else f"{value:.4f}"

    return {
        "test_point_id": tp.test_point_id,
        "description": tp.description,
        "start_s": f"{tp.start_s:g}",
        "end_s": f"{tp.end_s:g}",
        "status": result.status,
        "duration_flown_s": num(result.duration_flown_s),
        "nz_peak_g": num(result.nz_peak_g),
        "nz_mean_g": num(result.nz_mean_g),
        "airspeed_min_mps": num(result.airspeed_min_mps),
        "airspeed_max_mps": num(result.airspeed_max_mps),
        "alt_min_flown_m": num(result.alt_min_flown_m),
        "alt_max_flown_m": num(result.alt_max_flown_m),
        "bank_max_deg": num(result.bank_max_deg),
        "sink_rate_max_mps": num(result.sink_rate_max_mps),
        "exceedance_count": str(len(result.exceedances)),
        "exceedances": "; ".join(_exceedance_summary(e) for e in result.exceedances),
        "notes": "; ".join(result.notes),
    }


def write_outputs(results: Sequence[TestPointResult], log: FlightLog, log_path: str, card_path: str,
                  out_dir: str) -> Tuple[str, str]:
    os.makedirs(out_dir, exist_ok=True)
    md_path = os.path.join(out_dir, "report.md")
    csv_path = os.path.join(out_dir, "report.csv")
    with open(md_path, "w", encoding="utf-8") as handle:
        handle.write(render_markdown(results, log, log_path, card_path))
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=REPORT_CSV_COLUMNS)
        writer.writeheader()
        for result in results:
            writer.writerow(result_to_csv_row(result))
    return md_path, csv_path


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ulog", help="flight log (.ulg)")
    parser.add_argument("test_card", help="test card CSV")
    parser.add_argument("--out", required=True, help="output directory for report.md and report.csv")
    args = parser.parse_args(argv)

    for path in (args.ulog, args.test_card):
        if not os.path.isfile(path):
            parser.error(f"file not found: {path}")

    try:
        test_points = read_test_card(args.test_card)
    except TestCardError as exc:
        print(f"error: invalid test card: {exc}", file=sys.stderr)
        return 2

    log = FlightLog.from_file(args.ulog)
    results = reduce_test_card(test_points, log)
    write_outputs(results, log, args.ulog, args.test_card, args.out)
    print(summary_line(results, args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
