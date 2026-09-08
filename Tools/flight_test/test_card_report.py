#!/usr/bin/env python3
"""
Post-flight test-card data reduction for PX4 ULog files.

Reads a ULog and a test-card CSV and produces a per-test-point results report
(report.md and report.csv) with peak/mean Nz, airspeed range, altitude band,
max bank, max sink rate, duration flown and every limit exceedance.

Usage:
    python3 Tools/flight_test/test_card_report.py flight.ulg test_card.csv --out results/
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

G0 = 9.80665  # standard gravity [m/s^2]

STATUS_COMPLETE = "COMPLETE"
STATUS_EXCEEDANCE = "COMPLETE WITH EXCEEDANCE"
STATUS_NO_DATA = "NO DATA"

NOT_LOGGED = "not logged"
NO_DATA = "no data"

TEST_CARD_COLUMNS = (
    "test_point_id",
    "description",
    "start_s",
    "end_s",
    "nz_max_g",
    "airspeed_max_mps",
    "alt_min_m",
    "alt_max_m",
)
LIMIT_COLUMNS = TEST_CARD_COLUMNS[4:]

TOPIC_ACCEL = "vehicle_acceleration"
TOPIC_AIRSPEED = "airspeed_validated"
TOPIC_LOCAL_POS = "vehicle_local_position"
TOPIC_AIR_DATA = "vehicle_air_data"
TOPIC_ATTITUDE = "vehicle_attitude"
ULOG_TOPICS = (TOPIC_ACCEL, TOPIC_AIRSPEED, TOPIC_LOCAL_POS, TOPIC_AIR_DATA, TOPIC_ATTITUDE)

REPORT_CSV_COLUMNS = (
    "test_point_id",
    "description",
    "status",
    "start_s",
    "end_s",
    "duration_flown_s",
    "nz_peak_g",
    "nz_mean_g",
    "airspeed_min_mps",
    "airspeed_max_mps",
    "alt_min_m",
    "alt_max_m",
    "bank_max_deg",
    "sink_rate_max_mps",
    "exceedance_count",
    "exceedances",
    "notes",
)

_TEST_POINT_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.\-]{0,63}$")
_MAX_DESCRIPTION_LEN = 255
_MAX_TIME_S = 7 * 24 * 3600.0


class TestCardError(ValueError):
    """Raised for a malformed or inconsistent test card."""


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

    def limit(self, name: str) -> Optional[float]:
        return getattr(self, name)


def _parse_float(text: str, column: str, row_no: int) -> float:
    try:
        value = float(text.strip())
    except ValueError:
        raise TestCardError("row %d: column '%s' must be numeric, got %r" % (row_no, column, text))
    if not math.isfinite(value):
        raise TestCardError("row %d: column '%s' must be finite, got %r" % (row_no, column, text))
    return value


def _parse_optional_float(text: Optional[str], column: str, row_no: int) -> Optional[float]:
    if text is None or text.strip() == "":
        return None
    return _parse_float(text, column, row_no)


def load_test_card(path: str) -> List[TestPoint]:
    """Load and validate a test-card CSV. Raises TestCardError on bad input."""
    with open(path, newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        header = reader.fieldnames
        if header is None:
            raise TestCardError("test card is empty")
        header = [h.strip() for h in header]
        reader.fieldnames = header
        missing = [c for c in TEST_CARD_COLUMNS if c not in header]
        if missing:
            raise TestCardError("test card is missing column(s): %s" % ", ".join(missing))
        extra = [c for c in header if c not in TEST_CARD_COLUMNS]
        if extra:
            raise TestCardError("test card has unexpected column(s): %s" % ", ".join(extra))
        duplicates = sorted({c for c in header if header.count(c) > 1})
        if duplicates:
            raise TestCardError("test card has duplicate column(s): %s" % ", ".join(duplicates))

        points: List[TestPoint] = []
        seen_ids = set()
        for row_no, raw in enumerate(reader, start=2):
            surplus = raw.pop(None, None)  # DictReader collects fields beyond the header here
            if surplus and any(s.strip() for s in surplus):
                raise TestCardError("row %d: has %d field(s) more than the header" % (row_no, len(surplus)))
            row = {k: (v or "") for k, v in raw.items()}
            if all(v.strip() == "" for v in row.values()):
                continue

            point_id = row["test_point_id"].strip()
            if not _TEST_POINT_ID_RE.match(point_id):
                raise TestCardError(
                    "row %d: test_point_id %r must be 1-64 characters of [A-Za-z0-9_.-] "
                    "and start with a letter or digit" % (row_no, point_id))
            if point_id in seen_ids:
                raise TestCardError("row %d: duplicate test_point_id %r" % (row_no, point_id))
            seen_ids.add(point_id)

            description = row["description"].strip()
            if len(description) > _MAX_DESCRIPTION_LEN:
                raise TestCardError("row %d: description longer than %d characters"
                                    % (row_no, _MAX_DESCRIPTION_LEN))

            start_s = _parse_float(row["start_s"], "start_s", row_no)
            end_s = _parse_float(row["end_s"], "end_s", row_no)
            if start_s < 0 or end_s > _MAX_TIME_S:
                raise TestCardError("row %d: start_s/end_s must be within [0, %g] s" % (row_no, _MAX_TIME_S))
            if end_s <= start_s:
                raise TestCardError("row %d: end_s (%g) must be greater than start_s (%g)"
                                    % (row_no, end_s, start_s))

            limits: Dict[str, Optional[float]] = {}
            for column in LIMIT_COLUMNS:
                limits[column] = _parse_optional_float(row.get(column), column, row_no)
            for column in ("nz_max_g", "airspeed_max_mps"):
                if limits[column] is not None and limits[column] <= 0:
                    raise TestCardError("row %d: %s must be positive" % (row_no, column))
            if (limits["alt_min_m"] is not None and limits["alt_max_m"] is not None
                    and limits["alt_min_m"] >= limits["alt_max_m"]):
                raise TestCardError("row %d: alt_min_m must be below alt_max_m" % row_no)

            points.append(TestPoint(point_id, description, start_s, end_s, **limits))

    if not points:
        raise TestCardError("test card contains no test points")
    return points


@dataclass
class Channel:
    """A scalar time series. ``t`` is seconds in the report time reference.

    Invalid samples (NaN) are kept so that they bound the observed interval of
    the sample before them; ``dt`` is the observed interval attributed to each
    sample, i.e. the time to the next sample (valid or not), clipped to the
    window the channel was cut to. The last sample of a series has no observed
    successor and is attributed zero duration.

    A window is sample-and-hold: the last sample before ``start_s`` whose
    interval reaches into the window is carried in as the value held at
    ``start_s`` (timestamped ``start_s``, interval clipped to the window).
    """
    name: str
    t: np.ndarray
    v: np.ndarray
    dt: Optional[np.ndarray] = None

    def __post_init__(self):
        self.t = np.asarray(self.t, dtype=float)
        self.v = np.asarray(self.v, dtype=float)
        if self.dt is None:
            self.dt = self._intervals(self.t, np.inf)
        else:
            self.dt = np.asarray(self.dt, dtype=float)

    @staticmethod
    def _intervals(t: np.ndarray, end_s: float) -> np.ndarray:
        if len(t) == 0:
            return np.zeros(0)
        following = np.append(t[1:], t[-1])
        return np.maximum(np.minimum(following, end_s) - t, 0.0)

    def __len__(self) -> int:
        return int(np.count_nonzero(self.valid))

    @property
    def valid(self) -> np.ndarray:
        return np.isfinite(self.v)

    @property
    def values(self) -> np.ndarray:
        """Valid sample values only."""
        return self.v[self.valid]

    @property
    def times(self) -> np.ndarray:
        """Times of valid samples only."""
        return self.t[self.valid]

    def window(self, start_s: float, end_s: float) -> "Channel":
        t = self.t.copy()
        dt = self._intervals(t, end_s)
        mask = (t >= start_s) & (t <= end_s)
        before = np.flatnonzero(t < start_s)
        if len(before) and before[-1] + 1 < len(t) and t[before[-1] + 1] > start_s:
            held = before[-1]
            mask[held] = True
            dt[held] = max(min(t[held + 1], end_s) - start_s, 0.0)
            t[held] = start_s
        return Channel(self.name, t[mask], self.v[mask], dt[mask])


@dataclass
class FlightData:
    """Reduced channels extracted from a ULog, all in a common time reference."""
    source: str
    time_origin: str
    t0_us: int
    nz_g: Optional[Channel] = None
    cas_mps: Optional[Channel] = None
    alt_m: Optional[Channel] = None
    vz_down_mps: Optional[Channel] = None
    bank_deg: Optional[Channel] = None
    cas_source: str = NOT_LOGGED
    alt_source: str = NOT_LOGGED
    vz_source: str = NOT_LOGGED
    topics_present: Dict[str, bool] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    def channels(self) -> List[Channel]:
        return [c for c in (self.nz_g, self.cas_mps, self.alt_m, self.vz_down_mps, self.bank_deg)
                if c is not None]

    def data_span(self) -> Optional[Tuple[float, float]]:
        channels = [c for c in self.channels() if len(c)]
        if not channels:
            return None
        return (min(float(c.times[0]) for c in channels), max(float(c.times[-1]) for c in channels))


def _dataset(ulog: ULog, topic: str, notes: List[str]):
    """Return the lowest multi_id instance of ``topic`` (normally 0), else None.

    Appends a note when the instance used is not 0 so the report discloses it.
    """
    candidates = [d for d in ulog.data_list if d.name == topic]
    if not candidates:
        return None
    chosen = min(candidates, key=lambda d: d.multi_id)
    if chosen.multi_id != 0:
        notes.append("%s: instance 0 not logged, using instance %d" % (topic, chosen.multi_id))
    return chosen


def _has_field(dataset, name: str) -> bool:
    return any(f.field_name == name for f in dataset.field_data)


def quaternion_to_roll_deg(q: np.ndarray) -> np.ndarray:
    """Roll (bank) angle in degrees from PX4 [w, x, y, z] quaternions (FRD body to NED)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return np.degrees(roll)


def load_flight_data(ulog_path: str, time_origin: str = "log") -> FlightData:
    """Extract Nz, CAS, altitude, vertical speed and bank from a ULog."""
    if time_origin not in ("log", "boot"):
        raise ValueError("time_origin must be 'log' or 'boot'")

    ulog = ULog(ulog_path, message_name_filter_list=list(ULOG_TOPICS))
    t0_us = int(ulog.start_timestamp) if time_origin == "log" else 0
    fd = FlightData(source=os.path.basename(ulog_path), time_origin=time_origin, t0_us=t0_us)

    def seconds(ds) -> np.ndarray:
        return (ds.data["timestamp"].astype(np.int64) - t0_us) / 1e6

    accel = _dataset(ulog, TOPIC_ACCEL, fd.notes)
    fd.topics_present[TOPIC_ACCEL] = accel is not None
    if accel is not None:
        # xyz is specific force in the FRD body frame; level 1 g flight reads z = -g0.
        fd.nz_g = Channel("nz_g", seconds(accel), -accel.data["xyz[2]"] / G0)

    airspeed = _dataset(ulog, TOPIC_AIRSPEED, fd.notes)
    fd.topics_present[TOPIC_AIRSPEED] = airspeed is not None
    if airspeed is not None:
        # Logs from before the IAS/CAS rename only carry indicated_airspeed_m_s.
        cas_field = "calibrated_airspeed_m_s"
        if not _has_field(airspeed, cas_field):
            cas_field = "indicated_airspeed_m_s"
        fd.cas_mps = Channel("cas_mps", seconds(airspeed), airspeed.data[cas_field])
        fd.cas_source = "%s (%s)" % (TOPIC_AIRSPEED, cas_field)

    local_pos = _dataset(ulog, TOPIC_LOCAL_POS, fd.notes)
    air_data = _dataset(ulog, TOPIC_AIR_DATA, fd.notes)
    fd.topics_present[TOPIC_LOCAL_POS] = local_pos is not None
    fd.topics_present[TOPIC_AIR_DATA] = air_data is not None
    if local_pos is not None:
        t = seconds(local_pos)
        z = local_pos.data["z"].astype(float)
        z_valid = local_pos.data["z_valid"].astype(bool)
        if _has_field(local_pos, "z_global") and _has_field(local_pos, "ref_alt"):
            z_global = local_pos.data["z_global"].astype(bool)
        else:
            z_global = np.zeros(len(z), dtype=bool)
        if np.any(z_global):
            # ref_alt is only meaningful while z_global is set; samples without a
            # global reference are excluded rather than mixed into the AMSL band.
            alt = np.where(z_global, local_pos.data["ref_alt"].astype(float) - z, np.nan)
            fd.alt_source = "%s (ref_alt - z, AMSL)" % TOPIC_LOCAL_POS
            n_relative = int(np.count_nonzero(~z_global))
            if n_relative:
                fd.notes.append("%s: %d of %d samples have no global altitude reference (z_global false) "
                                "and were excluded from altitude" % (TOPIC_LOCAL_POS, n_relative, len(z)))
        else:
            alt = -z
            fd.alt_source = "%s (-z, relative to local origin)" % TOPIC_LOCAL_POS
        alt = np.where(z_valid, alt, np.nan)
        fd.alt_m = Channel("alt_m", t, alt)

        vz = local_pos.data["vz"].astype(float)
        vz = np.where(local_pos.data["v_z_valid"].astype(bool), vz, np.nan)
        fd.vz_down_mps = Channel("vz_down_mps", t, vz)
        fd.vz_source = "%s (vz)" % TOPIC_LOCAL_POS
    elif air_data is not None:
        t = seconds(air_data)
        alt = air_data.data["baro_alt_meter"].astype(float)
        fd.alt_m = Channel("alt_m", t, alt)
        fd.alt_source = "%s (baro_alt_meter, AMSL)" % TOPIC_AIR_DATA
        if len(fd.alt_m) >= 2:
            fd.vz_down_mps = Channel("vz_down_mps", fd.alt_m.times, -np.gradient(fd.alt_m.values, fd.alt_m.times))
            fd.vz_source = "%s (d/dt baro_alt_meter)" % TOPIC_AIR_DATA

    attitude = _dataset(ulog, TOPIC_ATTITUDE, fd.notes)
    fd.topics_present[TOPIC_ATTITUDE] = attitude is not None
    if attitude is not None:
        q = np.column_stack([attitude.data["q[%d]" % i] for i in range(4)]).astype(float)
        fd.bank_deg = Channel("bank_deg", seconds(attitude), np.abs(quaternion_to_roll_deg(q)))

    return fd


@dataclass(frozen=True)
class Exceedance:
    parameter: str
    kind: str  # "max" (value above limit) or "min" (value below limit)
    limit: float
    peak: float
    seconds_outside: float
    first_time_s: float

    def describe(self) -> str:
        return "%s %s limit %.2f, peak %.2f, %.2f s outside, first at %.2f s" % (
            self.parameter, self.kind, self.limit, self.peak, self.seconds_outside, self.first_time_s)


@dataclass
class TestPointResult:
    point: TestPoint
    status: str
    duration_flown_s: float
    nz_peak_g: Optional[float] = None
    nz_mean_g: Optional[float] = None
    airspeed_min_mps: Optional[float] = None
    airspeed_max_mps: Optional[float] = None
    alt_min_m: Optional[float] = None
    alt_max_m: Optional[float] = None
    bank_max_deg: Optional[float] = None
    sink_rate_max_mps: Optional[float] = None
    exceedances: List[Exceedance] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    availability: Dict[str, str] = field(default_factory=dict)

    def metric_text(self, name: str, fmt: str = "%.2f", unit: str = "") -> str:
        value = getattr(self, name)
        if value is None:
            return self.availability.get(name, NO_DATA)
        return _format_number(value, fmt) + unit


def _format_number(value: float, fmt: str) -> str:
    text = fmt % value
    if float(text) == 0.0:
        text = fmt % 0.0  # avoid printing -0.0
    return text


def find_exceedance(channel: Channel, parameter: str, kind: str, limit: float) -> Optional[Exceedance]:
    """Evaluate ``channel`` against ``limit``; return an Exceedance or None."""
    if len(channel) == 0:
        return None
    values = channel.values
    with np.errstate(invalid="ignore"):
        if kind == "max":
            outside = channel.valid & (channel.v > limit)
            peak = float(np.max(values))
        elif kind == "min":
            outside = channel.valid & (channel.v < limit)
            peak = float(np.min(values))
        else:
            raise ValueError("kind must be 'max' or 'min'")
    if not np.any(outside):
        return None
    seconds_outside = float(np.sum(channel.dt[outside]))
    first_time_s = float(channel.t[np.argmax(outside)])
    return Exceedance(parameter, kind, float(limit), peak, seconds_outside, first_time_s)


def reduce_test_point(fd: FlightData, point: TestPoint) -> TestPointResult:
    """Compute metrics, exceedances and status for one test point."""
    span = fd.data_span()
    if span is None:
        duration = 0.0
    else:
        duration = max(0.0, min(point.end_s, span[1]) - max(point.start_s, span[0]))
    result = TestPointResult(point=point, status=STATUS_NO_DATA, duration_flown_s=duration)

    def windowed(channel: Optional[Channel], metrics: Sequence[str]) -> Optional[Channel]:
        if channel is None:
            for m in metrics:
                result.availability[m] = NOT_LOGGED
            return None
        win = channel.window(point.start_s, point.end_s)
        if len(win) == 0:
            for m in metrics:
                result.availability[m] = NO_DATA
            return None
        return win

    any_data = False

    nz = windowed(fd.nz_g, ("nz_peak_g", "nz_mean_g"))
    if nz is not None:
        any_data = True
        result.nz_peak_g = float(np.max(nz.values))
        result.nz_mean_g = float(np.mean(nz.values))

    cas = windowed(fd.cas_mps, ("airspeed_min_mps", "airspeed_max_mps"))
    if cas is not None:
        any_data = True
        result.airspeed_min_mps = float(np.min(cas.values))
        result.airspeed_max_mps = float(np.max(cas.values))

    alt = windowed(fd.alt_m, ("alt_min_m", "alt_max_m"))
    if alt is not None:
        any_data = True
        result.alt_min_m = float(np.min(alt.values))
        result.alt_max_m = float(np.max(alt.values))

    vz = windowed(fd.vz_down_mps, ("sink_rate_max_mps",))
    if vz is not None:
        any_data = True
        result.sink_rate_max_mps = float(np.max(vz.values))

    bank = windowed(fd.bank_deg, ("bank_max_deg",))
    if bank is not None:
        any_data = True
        result.bank_max_deg = float(np.max(bank.values))

    checks = (
        ("nz_max_g", nz, "Nz [g]", "max", "nz_peak_g"),
        ("airspeed_max_mps", cas, "CAS [m/s]", "max", "airspeed_max_mps"),
        ("alt_min_m", alt, "altitude [m]", "min", "alt_min_m"),
        ("alt_max_m", alt, "altitude [m]", "max", "alt_max_m"),
    )
    for limit_name, channel, label, kind, metric in checks:
        limit = point.limit(limit_name)
        if limit is None:
            continue
        if channel is None:
            result.notes.append("%s limit %.2f not evaluated (%s)"
                                % (limit_name, limit, result.availability.get(metric, NO_DATA)))
            continue
        exc = find_exceedance(channel, label, kind, limit)
        if exc is not None:
            result.exceedances.append(exc)

    if not any_data:
        result.status = STATUS_NO_DATA
        result.notes = ["no samples logged between %.2f s and %.2f s" % (point.start_s, point.end_s)]
    elif result.exceedances:
        result.status = STATUS_EXCEEDANCE
    else:
        result.status = STATUS_COMPLETE
    return result


def reduce_test_card(fd: FlightData, points: Sequence[TestPoint]) -> List[TestPointResult]:
    return [reduce_test_point(fd, p) for p in points]


def _md_escape(text: str) -> str:
    return text.replace("|", "\\|").replace("\n", " ")


def write_report_md(results: Sequence[TestPointResult], fd: FlightData, card_path: str, out_path: str) -> None:
    lines: List[str] = []
    lines.append("# Flight Test Card Report")
    lines.append("")
    lines.append("- Log: `%s`" % fd.source)
    lines.append("- Test card: `%s`" % os.path.basename(card_path))
    lines.append("- Time reference: seconds since %s" % ("log start" if fd.time_origin == "log" else "boot"))
    lines.append("- Airspeed source: %s" % fd.cas_source)
    lines.append("- Altitude source: %s" % fd.alt_source)
    lines.append("- Vertical speed source: %s" % fd.vz_source)
    absent = [t for t in ULOG_TOPICS if not fd.topics_present.get(t, False)]
    if absent:
        lines.append("- Topics not logged: %s" % ", ".join("`%s`" % t for t in absent))
    for note in fd.notes:
        lines.append("- Note: %s" % note)
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(summary_line(results))
    lines.append("")
    lines.append("| Test point | Description | Status | Flown [s] | Nz peak [g] | Nz mean [g] | CAS min/max [m/s] "
                 "| Alt min/max [m] | Bank max [deg] | Sink max [m/s] | Exceedances |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in results:
        lines.append("| %s | %s | %s | %.1f | %s | %s | %s / %s | %s / %s | %s | %s | %d |" % (
            _md_escape(r.point.test_point_id), _md_escape(r.point.description), r.status,
            r.duration_flown_s, r.metric_text("nz_peak_g"), r.metric_text("nz_mean_g"),
            r.metric_text("airspeed_min_mps", "%.1f"), r.metric_text("airspeed_max_mps", "%.1f"),
            r.metric_text("alt_min_m", "%.1f"), r.metric_text("alt_max_m", "%.1f"),
            r.metric_text("bank_max_deg", "%.1f"), r.metric_text("sink_rate_max_mps", "%.1f"),
            len(r.exceedances)))
    lines.append("")

    lines.append("## Test Points")
    for r in results:
        p = r.point
        lines.append("")
        lines.append("### %s: %s" % (_md_escape(p.test_point_id), _md_escape(p.description)))
        lines.append("")
        lines.append("- Status: **%s**" % r.status)
        lines.append("- Window: %.2f s to %.2f s (planned %.2f s, flown %.2f s)"
                     % (p.start_s, p.end_s, p.end_s - p.start_s, r.duration_flown_s))
        lines.append("- Nz: peak %s, mean %s"
                     % (r.metric_text("nz_peak_g", unit=" g"), r.metric_text("nz_mean_g", unit=" g")))
        lines.append("- CAS: min %s, max %s"
                     % (r.metric_text("airspeed_min_mps", unit=" m/s"), r.metric_text("airspeed_max_mps", unit=" m/s")))
        lines.append("- Altitude: min %s, max %s"
                     % (r.metric_text("alt_min_m", unit=" m"), r.metric_text("alt_max_m", unit=" m")))
        lines.append("- Bank: max %s" % r.metric_text("bank_max_deg", "%.1f", " deg"))
        lines.append("- Sink rate: max %s" % r.metric_text("sink_rate_max_mps", unit=" m/s"))
        limits = ["%s=%g" % (c, p.limit(c)) for c in LIMIT_COLUMNS if p.limit(c) is not None]
        lines.append("- Limits: %s" % (", ".join(limits) if limits else "none"))
        if r.exceedances:
            lines.append("")
            lines.append("| Parameter | Limit | Peak | Seconds outside | First exceedance [s] |")
            lines.append("|---|---|---|---|---|")
            for e in r.exceedances:
                lines.append("| %s (%s) | %.2f | %.2f | %.2f | %.2f |"
                             % (e.parameter, e.kind, e.limit, e.peak, e.seconds_outside, e.first_time_s))
        for note in r.notes:
            lines.append("- Note: %s" % note)
    lines.append("")

    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


def _csv_value(result: TestPointResult, name: str) -> str:
    value = getattr(result, name)
    if value is None:
        return result.availability.get(name, NO_DATA)
    return _format_number(value, "%.3f")


def write_report_csv(results: Sequence[TestPointResult], out_path: str) -> None:
    with open(out_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=REPORT_CSV_COLUMNS)
        writer.writeheader()
        for r in results:
            writer.writerow({
                "test_point_id": r.point.test_point_id,
                "description": r.point.description,
                "status": r.status,
                "start_s": "%.3f" % r.point.start_s,
                "end_s": "%.3f" % r.point.end_s,
                "duration_flown_s": "%.3f" % r.duration_flown_s,
                "nz_peak_g": _csv_value(r, "nz_peak_g"),
                "nz_mean_g": _csv_value(r, "nz_mean_g"),
                "airspeed_min_mps": _csv_value(r, "airspeed_min_mps"),
                "airspeed_max_mps": _csv_value(r, "airspeed_max_mps"),
                "alt_min_m": _csv_value(r, "alt_min_m"),
                "alt_max_m": _csv_value(r, "alt_max_m"),
                "bank_max_deg": _csv_value(r, "bank_max_deg"),
                "sink_rate_max_mps": _csv_value(r, "sink_rate_max_mps"),
                "exceedance_count": str(len(r.exceedances)),
                "exceedances": "; ".join(e.describe() for e in r.exceedances),
                "notes": "; ".join(r.notes),
            })


def summary_line(results: Sequence[TestPointResult]) -> str:
    counts = {STATUS_COMPLETE: 0, STATUS_EXCEEDANCE: 0, STATUS_NO_DATA: 0}
    for r in results:
        counts[r.status] += 1
    return "%d test points: %d %s, %d %s, %d %s" % (
        len(results),
        counts[STATUS_COMPLETE], STATUS_COMPLETE,
        counts[STATUS_EXCEEDANCE], STATUS_EXCEEDANCE,
        counts[STATUS_NO_DATA], STATUS_NO_DATA)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Reduce a PX4 ULog against a flight-test card into a per-test-point report.")
    parser.add_argument("ulog", metavar="flight.ulg", help="ULog input file")
    parser.add_argument("test_card", metavar="test_card.csv",
                        help="test card CSV with columns %s" % ", ".join(TEST_CARD_COLUMNS))
    parser.add_argument("--out", default=".", metavar="dir",
                        help="output directory for report.md and report.csv (default: current directory)")
    parser.add_argument("--time-origin", choices=("log", "boot"), default="log",
                        help="reference for test-card start_s/end_s: first logged sample (log, default) "
                             "or autopilot boot (boot)")
    return parser


def run(ulog_path: str, card_path: str, out_dir: str, time_origin: str = "log") -> List[TestPointResult]:
    points = load_test_card(card_path)
    fd = load_flight_data(ulog_path, time_origin=time_origin)
    results = reduce_test_card(fd, points)
    os.makedirs(out_dir, exist_ok=True)
    write_report_md(results, fd, card_path, os.path.join(out_dir, "report.md"))
    write_report_csv(results, os.path.join(out_dir, "report.csv"))
    return results


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    for path, label in ((args.ulog, "ULog"), (args.test_card, "test card")):
        if not os.path.isfile(path):
            print("error: %s file not found: %s" % (label, path), file=sys.stderr)
            return 2
    try:
        results = run(args.ulog, args.test_card, args.out, args.time_origin)
    except TestCardError as exc:
        print("error: invalid test card: %s" % exc, file=sys.stderr)
        return 2
    except (ValueError, TypeError, OSError) as exc:
        # pyulog raises TypeError for files that are not ULogs
        print("error: could not process inputs: %s" % exc, file=sys.stderr)
        return 1
    print("%s -> %s" % (summary_line(results), os.path.join(args.out, "report.md")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
