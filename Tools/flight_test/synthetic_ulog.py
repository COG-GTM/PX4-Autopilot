#!/usr/bin/env python3
"""
Generate a synthetic PX4 ULog for exercising test_card_report.py without a flight.

The flight profile is a list of keyframes (time, Nz, CAS, altitude, bank); every
channel is linearly interpolated between keyframes and written to the uORB
topics the report tool consumes. Topics can be left out to simulate a log
where a sensor or estimator output was not recorded.

Usage:
    python3 Tools/flight_test/synthetic_ulog.py demo_flight.ulg
    python3 Tools/flight_test/synthetic_ulog.py no_airspeed.ulg --without airspeed_validated
"""

import argparse
import struct
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

G0 = 9.80665

TOPIC_ACCEL = "vehicle_acceleration"
TOPIC_AIRSPEED = "airspeed_validated"
TOPIC_LOCAL_POS = "vehicle_local_position"
TOPIC_AIR_DATA = "vehicle_air_data"
TOPIC_ATTITUDE = "vehicle_attitude"
ALL_TOPICS = (TOPIC_ACCEL, TOPIC_AIRSPEED, TOPIC_LOCAL_POS, TOPIC_AIR_DATA, TOPIC_ATTITUDE)

DEFAULT_RATES_HZ = {
    TOPIC_ACCEL: 200.0,
    TOPIC_AIRSPEED: 10.0,
    TOPIC_LOCAL_POS: 50.0,
    TOPIC_AIR_DATA: 20.0,
    TOPIC_ATTITUDE: 100.0,
}

# ULog message formats (subset of the real uORB messages; field order follows
# msg/*.msg so that pyulog exposes the same field names). Types use the ULog
# on-disk names (uint64_t, float, ...), not the .msg names.
MESSAGE_FORMATS = {
    TOPIC_ACCEL: (
        ("uint64_t", "timestamp"),
        ("uint64_t", "timestamp_sample"),
        ("float[3]", "xyz"),
    ),
    TOPIC_AIRSPEED: (
        ("uint64_t", "timestamp"),
        ("float", "indicated_airspeed_m_s"),
        ("float", "calibrated_airspeed_m_s"),
        ("float", "true_airspeed_m_s"),
        ("int8_t", "airspeed_source"),
    ),
    TOPIC_LOCAL_POS: (
        ("uint64_t", "timestamp"),
        ("uint64_t", "timestamp_sample"),
        ("bool", "xy_valid"),
        ("bool", "z_valid"),
        ("bool", "v_xy_valid"),
        ("bool", "v_z_valid"),
        ("float", "x"),
        ("float", "y"),
        ("float", "z"),
        ("float", "vx"),
        ("float", "vy"),
        ("float", "vz"),
        ("bool", "xy_global"),
        ("bool", "z_global"),
        ("uint64_t", "ref_timestamp"),
        ("double", "ref_lat"),
        ("double", "ref_lon"),
        ("float", "ref_alt"),
    ),
    TOPIC_AIR_DATA: (
        ("uint64_t", "timestamp"),
        ("uint64_t", "timestamp_sample"),
        ("uint32_t", "baro_device_id"),
        ("float", "baro_alt_meter"),
        ("float", "baro_pressure_pa"),
    ),
    TOPIC_ATTITUDE: (
        ("uint64_t", "timestamp"),
        ("uint64_t", "timestamp_sample"),
        ("float[4]", "q"),
        ("float[4]", "delta_q_reset"),
        ("uint8_t", "quat_reset_counter"),
    ),
}

# airspeed_validated as logged before calibrated_airspeed_m_s existed (IAS only).
LEGACY_AIRSPEED_FORMAT = tuple(f for f in MESSAGE_FORMATS[TOPIC_AIRSPEED] if f[1] != "calibrated_airspeed_m_s")

_STRUCT_CODES = {
    "int8_t": "b", "uint8_t": "B", "int16_t": "h", "uint16_t": "H", "int32_t": "i", "uint32_t": "I",
    "int64_t": "q", "uint64_t": "Q", "float": "f", "double": "d", "bool": "?", "char": "c",
}

_ULOG_MAGIC = b"ULog\x01\x12\x35"
_ULOG_VERSION = 1
_MSG_FLAG_BITS = ord("B")
_MSG_FORMAT = ord("F")
_MSG_INFO = ord("I")
_MSG_ADD_LOGGED = ord("A")
_MSG_DATA = ord("D")


@dataclass(frozen=True)
class Keyframe:
    """Flight state at time ``t_s`` (seconds since log start)."""
    t_s: float
    nz_g: float = 1.0
    cas_mps: float = 25.0
    alt_m: float = 500.0
    bank_deg: float = 0.0


# Demo profile matching example_test_card.csv. Log start is t = 0.
DEMO_KEYFRAMES = (
    Keyframe(0.0, 1.0, 22.0, 500.0, 0.0),
    Keyframe(45.0, 1.0, 22.0, 500.0, 0.0),
    # TP-02: symmetric pull-up peaking at 2.8 g (card limit 2.5 g)
    Keyframe(48.0, 1.0, 28.0, 500.0, 0.0),
    Keyframe(52.0, 2.8, 28.0, 510.0, 0.0),
    Keyframe(56.0, 1.0, 26.0, 520.0, 0.0),
    Keyframe(65.0, 1.0, 24.0, 500.0, 0.0),
    # TP-03: 45 deg bank level turn
    Keyframe(70.0, 1.4, 24.0, 500.0, 45.0),
    Keyframe(85.0, 1.4, 24.0, 500.0, 45.0),
    Keyframe(90.0, 1.0, 24.0, 500.0, 0.0),
    Keyframe(95.0, 1.0, 24.0, 600.0, 0.0),
    # TP-04: dive to 38 m/s CAS bottoming at 420 m (card limits 35 m/s, 450 m)
    Keyframe(105.0, 0.8, 38.0, 420.0, 0.0),
    Keyframe(112.0, 1.6, 30.0, 450.0, 0.0),
    Keyframe(120.0, 1.0, 24.0, 460.0, 0.0),
    Keyframe(130.0, 1.0, 22.0, 460.0, 0.0),
)


def _pack_message(msg_type: int, payload: bytes) -> bytes:
    return struct.pack("<HB", len(payload), msg_type) + payload


def _format_string(name: str, fields: Sequence[Tuple[str, str]]) -> str:
    return name + ":" + "".join("%s %s;" % (t, f) for t, f in fields)


def _struct_format(fields: Sequence[Tuple[str, str]]) -> str:
    fmt = "<"
    for type_str, _ in fields:
        if "[" in type_str:
            base, count = type_str[:-1].split("[")
            fmt += _STRUCT_CODES[base] * int(count)
        else:
            fmt += _STRUCT_CODES[type_str]
    return fmt


class ULogWriter:
    """Minimal ULog writer: header, flag bits, formats, info, subscriptions, data."""

    def __init__(self, start_timestamp_us: int):
        self._definitions = bytearray()
        self._data = bytearray()
        self._formats: Dict[str, Sequence[Tuple[str, str]]] = {}
        self._subscriptions: Dict[str, Tuple[int, str]] = {}
        self._start_timestamp_us = int(start_timestamp_us)
        self._definitions += _pack_message(_MSG_FLAG_BITS, bytes(40))

    def add_format(self, name: str, fields: Sequence[Tuple[str, str]]) -> None:
        self._formats[name] = tuple(fields)
        self._definitions += _pack_message(_MSG_FORMAT, _format_string(name, fields).encode("ascii"))

    def add_info(self, key: str, value: str) -> None:
        encoded = value.encode("ascii")
        key_str = ("char[%d] %s" % (len(encoded), key)).encode("ascii")
        self._definitions += _pack_message(_MSG_INFO, struct.pack("<B", len(key_str)) + key_str + encoded)

    def subscribe(self, name: str, multi_id: int = 0) -> int:
        if name not in self._formats:
            raise KeyError("no format registered for %s" % name)
        msg_id = len(self._subscriptions)
        self._subscriptions[name] = (msg_id, _struct_format(self._formats[name]))
        self._data += _pack_message(_MSG_ADD_LOGGED, struct.pack("<BH", multi_id, msg_id) + name.encode("ascii"))
        return msg_id

    def write_sample(self, name: str, *values) -> None:
        msg_id, fmt = self._subscriptions[name]
        self._data += _pack_message(_MSG_DATA, struct.pack("<H", msg_id) + struct.pack(fmt, *values))

    def to_bytes(self) -> bytes:
        header = _ULOG_MAGIC + struct.pack("<BQ", _ULOG_VERSION, self._start_timestamp_us)
        return header + bytes(self._definitions) + bytes(self._data)

    def save(self, path: str) -> None:
        with open(path, "wb") as handle:
            handle.write(self.to_bytes())


def _interp(t: np.ndarray, keyframes: Sequence[Keyframe], attr: str) -> np.ndarray:
    kt = np.array([k.t_s for k in keyframes], dtype=float)
    kv = np.array([getattr(k, attr) for k in keyframes], dtype=float)
    return np.interp(t, kt, kv)


def _sample_times(keyframes: Sequence[Keyframe], rate_hz: float) -> np.ndarray:
    t_start, t_end = keyframes[0].t_s, keyframes[-1].t_s
    n = int(np.floor((t_end - t_start) * rate_hz)) + 1
    return t_start + np.arange(n) / rate_hz


def roll_to_quaternion(roll_rad: np.ndarray) -> np.ndarray:
    """[w, x, y, z] quaternion for a pure roll rotation."""
    half = 0.5 * np.asarray(roll_rad, dtype=float)
    q = np.zeros((len(half), 4))
    q[:, 0] = np.cos(half)
    q[:, 1] = np.sin(half)
    return q


def build_synthetic_ulog(keyframes: Sequence[Keyframe] = DEMO_KEYFRAMES,
                         topics: Iterable[str] = ALL_TOPICS,
                         boot_offset_s: float = 30.0,
                         rates_hz: Optional[Dict[str, float]] = None,
                         ref_alt_m: float = 0.0,
                         z_global: bool = True,
                         z_global_from_s: Optional[float] = None,
                         legacy_airspeed: bool = False,
                         multi_id: int = 0) -> bytes:
    """Return the bytes of a ULog for ``keyframes``, logging only ``topics``.

    Timestamps are ``boot_offset_s`` plus the keyframe time, so the log's start
    timestamp (header) equals ``boot_offset_s`` and test-card times measured from
    log start equal keyframe times. Altitude in vehicle_local_position is
    encoded as ``z = ref_alt_m - alt`` when ``z_global`` else ``z = -alt``.
    With ``z_global_from_s`` the global reference is only available from that
    time on: earlier samples have ``z_global`` false and ``z = -alt`` (relative).
    ``legacy_airspeed`` writes airspeed_validated without calibrated_airspeed_m_s.
    ``multi_id`` is the uORB instance every topic is logged as.
    """
    if len(keyframes) < 2:
        raise ValueError("at least two keyframes are required")
    if any(b.t_s <= a.t_s for a, b in zip(keyframes, keyframes[1:])):
        raise ValueError("keyframes must have strictly increasing times")
    topics = tuple(topics)
    unknown = [t for t in topics if t not in MESSAGE_FORMATS]
    if unknown:
        raise ValueError("unknown topic(s): %s" % ", ".join(unknown))
    rates = dict(DEFAULT_RATES_HZ)
    if rates_hz:
        rates.update(rates_hz)

    boot_offset_us = int(round(boot_offset_s * 1e6))
    writer = ULogWriter(start_timestamp_us=boot_offset_us)
    writer.add_info("sys_name", "PX4")
    writer.add_info("ver_sw", "synthetic")
    for topic in topics:
        if topic == TOPIC_AIRSPEED and legacy_airspeed:
            writer.add_format(topic, LEGACY_AIRSPEED_FORMAT)
        else:
            writer.add_format(topic, MESSAGE_FORMATS[topic])
    for topic in topics:
        writer.subscribe(topic, multi_id)

    def stamps(t: np.ndarray) -> np.ndarray:
        return boot_offset_us + np.round(t * 1e6).astype(np.int64)

    samples: List[Tuple[int, str, tuple]] = []

    if TOPIC_ACCEL in topics:
        t = _sample_times(keyframes, rates[TOPIC_ACCEL])
        nz = _interp(t, keyframes, "nz_g")
        for ts, n in zip(stamps(t), nz):
            samples.append((int(ts), TOPIC_ACCEL, (int(ts), int(ts), 0.0, 0.0, float(-n * G0))))

    if TOPIC_AIRSPEED in topics:
        t = _sample_times(keyframes, rates[TOPIC_AIRSPEED])
        cas = _interp(t, keyframes, "cas_mps")
        for ts, c in zip(stamps(t), cas):
            if legacy_airspeed:
                samples.append((int(ts), TOPIC_AIRSPEED, (int(ts), float(c), float(c * 1.05), 0)))
            else:
                samples.append((int(ts), TOPIC_AIRSPEED, (int(ts), float(c), float(c), float(c * 1.05), 0)))

    if TOPIC_LOCAL_POS in topics:
        t = _sample_times(keyframes, rates[TOPIC_LOCAL_POS])
        alt = _interp(t, keyframes, "alt_m")
        vz_down = -np.gradient(alt, t)
        if z_global_from_s is not None:
            is_global = t >= z_global_from_s
        else:
            is_global = np.full(len(t), bool(z_global))
        z = np.where(is_global, ref_alt_m - alt, -alt)
        for ts, zi, vzi, gi in zip(stamps(t), z, vz_down, is_global):
            samples.append((int(ts), TOPIC_LOCAL_POS, (
                int(ts), int(ts), True, True, True, True,
                0.0, 0.0, float(zi), 0.0, 0.0, float(vzi),
                bool(gi), bool(gi), boot_offset_us, 47.397742, 8.545594, float(ref_alt_m))))

    if TOPIC_AIR_DATA in topics:
        t = _sample_times(keyframes, rates[TOPIC_AIR_DATA])
        alt = _interp(t, keyframes, "alt_m")
        for ts, a in zip(stamps(t), alt):
            pressure = 101325.0 * (1.0 - 2.25577e-5 * a) ** 5.25588
            samples.append((int(ts), TOPIC_AIR_DATA, (int(ts), int(ts), 1, float(a), float(pressure))))

    if TOPIC_ATTITUDE in topics:
        t = _sample_times(keyframes, rates[TOPIC_ATTITUDE])
        q = roll_to_quaternion(np.radians(_interp(t, keyframes, "bank_deg")))
        for ts, qi in zip(stamps(t), q):
            samples.append((int(ts), TOPIC_ATTITUDE, (int(ts), int(ts), *map(float, qi), 1.0, 0.0, 0.0, 0.0, 0)))

    samples.sort(key=lambda s: s[0])
    for _, topic, values in samples:
        writer.write_sample(topic, *values)
    return writer.to_bytes()


def write_synthetic_ulog(path: str, **kwargs) -> str:
    with open(path, "wb") as handle:
        handle.write(build_synthetic_ulog(**kwargs))
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Write a synthetic PX4 ULog for test_card_report.py.")
    parser.add_argument("output", metavar="out.ulg", help="output ULog path")
    parser.add_argument("--without", metavar="topic", action="append", default=[], choices=ALL_TOPICS,
                        help="omit a topic (repeatable), e.g. --without airspeed_validated")
    parser.add_argument("--boot-offset", type=float, default=30.0, metavar="s",
                        help="seconds between autopilot boot and log start (default: 30)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    topics = [t for t in ALL_TOPICS if t not in args.without]
    write_synthetic_ulog(args.output, topics=topics, boot_offset_s=args.boot_offset)
    print("wrote %s (%s)" % (args.output, ", ".join(topics)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
