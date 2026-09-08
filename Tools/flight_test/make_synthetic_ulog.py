#!/usr/bin/env python3
"""
Write a synthetic PX4 ULog for exercising the test-card report tool.

The scripted profile is: trim -> climb -> level acceleration -> wind-up turn
(which deliberately busts a 2.5 g Nz limit) -> roll out -> descent -> trim. Only the
topics and fields read by test_card_report.py are written, so the log is not
a substitute for real flight data.

The ULog binary format is written directly with `struct` (header, flag bits,
format definitions, info messages, subscriptions, data). pyulog can read ULog
files but its writer only serialises an object it parsed itself, so it is not
used here. The output is validated by reading it back with pyulog.

Usage:
    python3 Tools/flight_test/make_synthetic_ulog.py --out synthetic.ulg
    python3 Tools/flight_test/make_synthetic_ulog.py --out no_accel.ulg \\
        --skip-topic vehicle_acceleration
"""

import argparse
import math
import os
import struct
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

STANDARD_GRAVITY_MPS2 = 9.80665

# ULog message types (docs.px4.io/main/en/dev_log/ulog_file_format.html)
ULOG_MAGIC = b"\x55\x4c\x6f\x67\x01\x12\x35"
ULOG_VERSION = 1
MSG_TYPE_FLAG_BITS = ord("B")
MSG_TYPE_FORMAT = ord("F")
MSG_TYPE_INFO = ord("I")
MSG_TYPE_ADD_LOGGED_MSG = ord("A")
MSG_TYPE_DATA = ord("D")

STRUCT_FORMATS = {
    "int8_t": "b",
    "uint8_t": "B",
    "int16_t": "h",
    "uint16_t": "H",
    "int32_t": "i",
    "uint32_t": "I",
    "int64_t": "q",
    "uint64_t": "Q",
    "float": "f",
    "double": "d",
    "bool": "?",
}

# Log start (system uptime) is deliberately non-zero so that the report tool's
# "seconds from log start" handling is exercised.
LOG_START_TIMESTAMP_US = 100_000_000

HOME_ALT_M = 450.0
FAST_RATE_HZ = 50.0
SLOW_RATE_HZ = 10.0


@dataclass(frozen=True)
class Segment:
    """One scripted manoeuvre with linear ramps between start and end values."""

    name: str
    start_s: float
    end_s: float
    alt_start_m: float
    alt_end_m: float
    airspeed_start_mps: float
    airspeed_end_mps: float
    bank_start_deg: float
    bank_end_deg: float


# Continuous profile (no steps between segments). The wind-up turn ramps to
# 70 deg of bank, so Nz = 1/cos(bank) peaks at about 2.92 g before rolling out.
PROFILE: Tuple[Segment, ...] = (
    Segment("trim", 0.0, 20.0, 500.0, 500.0, 25.0, 25.0, 0.0, 0.0),
    Segment("climb", 20.0, 80.0, 500.0, 1100.0, 25.0, 22.0, 0.0, 0.0),
    Segment("level_accel", 80.0, 140.0, 1100.0, 1100.0, 22.0, 45.0, 0.0, 0.0),
    Segment("wind_up_turn", 140.0, 195.0, 1100.0, 1100.0, 45.0, 45.0, 0.0, 70.0),
    Segment("roll_out", 195.0, 200.0, 1100.0, 1100.0, 45.0, 45.0, 70.0, 0.0),
    Segment("descent", 200.0, 260.0, 1100.0, 500.0, 45.0, 30.0, 0.0, 0.0),
    Segment("trim_out", 260.0, 280.0, 500.0, 500.0, 30.0, 30.0, 0.0, 0.0),
)

PROFILE_DURATION_S = PROFILE[-1].end_s

ALL_TOPICS = (
    "vehicle_acceleration",
    "airspeed_validated",
    "vehicle_local_position",
    "vehicle_air_data",
    "vehicle_attitude",
)


def _interp_profile(t: np.ndarray) -> Dict[str, np.ndarray]:
    """Piecewise-linear altitude, airspeed and bank angle for sample times t."""
    knots_t: List[float] = []
    knots_alt: List[float] = []
    knots_ias: List[float] = []
    knots_bank: List[float] = []
    for seg in PROFILE:
        for time, alt, ias, bank in (
            (seg.start_s, seg.alt_start_m, seg.airspeed_start_mps, seg.bank_start_deg),
            (seg.end_s, seg.alt_end_m, seg.airspeed_end_mps, seg.bank_end_deg),
        ):
            if knots_t and math.isclose(knots_t[-1], time):
                if (knots_alt[-1], knots_ias[-1], knots_bank[-1]) != (alt, ias, bank):
                    raise ValueError(f"profile is discontinuous at t={time} s ({seg.name})")
                continue
            knots_t.append(time)
            knots_alt.append(alt)
            knots_ias.append(ias)
            knots_bank.append(bank)
    return {
        "alt_m": np.interp(t, knots_t, knots_alt),
        "airspeed_mps": np.interp(t, knots_t, knots_ias),
        "bank_deg": np.interp(t, knots_t, knots_bank),
    }


def _quaternion_from_euler(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Hamilton quaternion [w, x, y, z] for a ZYX Euler rotation (body FRD -> NED)."""
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return np.stack(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        axis=1,
    )


def build_profile(
    fast_rate_hz: float = FAST_RATE_HZ,
    slow_rate_hz: float = SLOW_RATE_HZ,
    noise_scale: float = 1.0,
    seed: int = 0,
) -> Dict[str, Dict[str, np.ndarray]]:
    """
    Return topic -> field -> samples for the scripted profile.

    Field names use pyulog's flattened convention (e.g. ``xyz[2]``, ``q[0]``).
    """
    rng = np.random.default_rng(seed)

    def noise(shape, sigma):
        return rng.normal(0.0, sigma * noise_scale, shape)

    def timestamps(t):
        return (LOG_START_TIMESTAMP_US + np.round(t * 1e6)).astype(np.uint64)

    t_fast = np.arange(0.0, PROFILE_DURATION_S, 1.0 / fast_rate_hz)
    t_slow = np.arange(0.0, PROFILE_DURATION_S, 1.0 / slow_rate_hz)
    fast = _interp_profile(t_fast)
    slow = _interp_profile(t_slow)

    bank_rad = np.radians(fast["bank_deg"])
    nz_g = 1.0 / np.cos(bank_rad)
    # FRD body z is down; the accelerometer reads -g along body z in 1 g flight.
    az = -nz_g * STANDARD_GRAVITY_MPS2 + noise(t_fast.shape, 0.05)
    ax = noise(t_fast.shape, 0.05)
    ay = noise(t_fast.shape, 0.05)

    # Coordinated-turn yaw rate integrates into a heading for the attitude topic.
    yaw_rate = STANDARD_GRAVITY_MPS2 * np.tan(bank_rad) / np.maximum(fast["airspeed_mps"], 1.0)
    yaw = np.cumsum(yaw_rate) / fast_rate_hz
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
    pitch = np.radians(2.0) + noise(t_fast.shape, 0.002)
    q = _quaternion_from_euler(bank_rad + noise(t_fast.shape, 0.002), pitch, yaw)

    alt_slow = slow["alt_m"] + noise(t_slow.shape, 0.2)
    vz_down = np.gradient(slow["alt_m"], t_slow) * -1.0 + noise(t_slow.shape, 0.05)
    airspeed = slow["airspeed_mps"] + noise(t_slow.shape, 0.1)

    zeros_slow_u8 = np.zeros(t_slow.shape, dtype=np.uint8)
    ones_slow_u8 = np.ones(t_slow.shape, dtype=np.uint8)

    topics: Dict[str, Dict[str, np.ndarray]] = {
        "vehicle_acceleration": {
            "timestamp": timestamps(t_fast),
            "timestamp_sample": timestamps(t_fast),
            "xyz[0]": ax,
            "xyz[1]": ay,
            "xyz[2]": az,
        },
        "vehicle_attitude": {
            "timestamp": timestamps(t_fast),
            "timestamp_sample": timestamps(t_fast),
            "q[0]": q[:, 0],
            "q[1]": q[:, 1],
            "q[2]": q[:, 2],
            "q[3]": q[:, 3],
            "delta_q_reset[0]": np.ones(t_fast.shape),
            "delta_q_reset[1]": np.zeros(t_fast.shape),
            "delta_q_reset[2]": np.zeros(t_fast.shape),
            "delta_q_reset[3]": np.zeros(t_fast.shape),
            "quat_reset_counter": np.zeros(t_fast.shape, dtype=np.uint8),
        },
        "airspeed_validated": {
            "timestamp": timestamps(t_slow),
            "indicated_airspeed_m_s": airspeed,
            "calibrated_airspeed_m_s": airspeed,
            "true_airspeed_m_s": airspeed * 1.05,
            "airspeed_source": ones_slow_u8.astype(np.int8),
        },
        "vehicle_local_position": {
            "timestamp": timestamps(t_slow),
            "timestamp_sample": timestamps(t_slow),
            "z_valid": ones_slow_u8,
            "v_z_valid": ones_slow_u8,
            "z": -(alt_slow - HOME_ALT_M),
            "vz": vz_down,
            "z_global": ones_slow_u8,
            "ref_alt": np.full(t_slow.shape, HOME_ALT_M),
            "dead_reckoning": zeros_slow_u8,
        },
        "vehicle_air_data": {
            "timestamp": timestamps(t_slow),
            "timestamp_sample": timestamps(t_slow),
            "baro_device_id": np.full(t_slow.shape, 1, dtype=np.uint32),
            "baro_alt_meter": alt_slow + noise(t_slow.shape, 0.3),
            "baro_pressure_pa": 101325.0 * np.power(1.0 - 2.25577e-5 * alt_slow, 5.25588),
            "ambient_temperature": np.full(t_slow.shape, 15.0),
            "rho": np.full(t_slow.shape, 1.225),
        },
    }
    return topics


# ULog type definitions for each synthetic topic. Field names/types match the
# PX4 message definitions in msg/; only a subset of fields is carried.
TOPIC_FORMATS: Dict[str, Sequence[Tuple[str, int, str]]] = {
    "vehicle_acceleration": (
        ("uint64_t", 0, "timestamp"),
        ("uint64_t", 0, "timestamp_sample"),
        ("float", 3, "xyz"),
    ),
    "vehicle_attitude": (
        ("uint64_t", 0, "timestamp"),
        ("uint64_t", 0, "timestamp_sample"),
        ("float", 4, "q"),
        ("float", 4, "delta_q_reset"),
        ("uint8_t", 0, "quat_reset_counter"),
    ),
    "airspeed_validated": (
        ("uint64_t", 0, "timestamp"),
        ("float", 0, "indicated_airspeed_m_s"),
        ("float", 0, "calibrated_airspeed_m_s"),
        ("float", 0, "true_airspeed_m_s"),
        ("int8_t", 0, "airspeed_source"),
    ),
    "vehicle_local_position": (
        ("uint64_t", 0, "timestamp"),
        ("uint64_t", 0, "timestamp_sample"),
        ("bool", 0, "z_valid"),
        ("bool", 0, "v_z_valid"),
        ("float", 0, "z"),
        ("float", 0, "vz"),
        ("bool", 0, "z_global"),
        ("float", 0, "ref_alt"),
        ("bool", 0, "dead_reckoning"),
    ),
    "vehicle_air_data": (
        ("uint64_t", 0, "timestamp"),
        ("uint64_t", 0, "timestamp_sample"),
        ("uint32_t", 0, "baro_device_id"),
        ("float", 0, "baro_alt_meter"),
        ("float", 0, "baro_pressure_pa"),
        ("float", 0, "ambient_temperature"),
        ("float", 0, "rho"),
    ),
}


def _flatten_fields(fields: Sequence[Tuple[str, int, str]]) -> List[Tuple[str, str]]:
    flat: List[Tuple[str, str]] = []
    for type_name, array_size, name in fields:
        if array_size > 0:
            flat.extend((f"{name}[{i}]", type_name) for i in range(array_size))
        else:
            flat.append((name, type_name))
    return flat


def _message(msg_type: int, payload: bytes) -> bytes:
    return struct.pack("<HB", len(payload), msg_type) + payload


def _info_message(key: str, value: str) -> bytes:
    type_key = f"char[{len(value)}] {key}".encode()
    return _message(MSG_TYPE_INFO, struct.pack("<B", len(type_key)) + type_key + value.encode())


def write_ulog(
    path: str,
    topics: Dict[str, Dict[str, np.ndarray]],
    formats: Dict[str, Sequence[Tuple[str, int, str]]],
    start_timestamp_us: int,
    info: Optional[Dict[str, str]] = None,
) -> None:
    """Serialise ``topics`` (topic -> flattened field -> samples) to a ULog file."""
    info = info if info is not None else {"sys_name": "PX4", "ver_sw": "synthetic"}

    out = bytearray()
    out += ULOG_MAGIC + struct.pack("<B", ULOG_VERSION) + struct.pack("<Q", start_timestamp_us)
    out += _message(MSG_TYPE_FLAG_BITS, bytes(8) + bytes(8) + struct.pack("<3Q", 0, 0, 0))

    for name in topics:
        fields = formats[name]
        field_defs = "".join(
            f"{type_name}[{size}] {field};" if size > 0 else f"{type_name} {field};"
            for type_name, size, field in fields
        )
        out += _message(MSG_TYPE_FORMAT, f"{name}:{field_defs}".encode())

    for key, value in info.items():
        out += _info_message(key, value)

    data_items: List[Tuple[int, bytes]] = []
    for msg_id, name in enumerate(topics):
        out += _message(MSG_TYPE_ADD_LOGGED_MSG, struct.pack("<BH", 0, msg_id) + name.encode())

        flat = _flatten_fields(formats[name])
        packer = struct.Struct("<" + "".join(STRUCT_FORMATS[t] for _, t in flat))
        columns = [topics[name][field] for field, _ in flat]
        stamps = topics[name]["timestamp"]
        for i in range(len(stamps)):
            row = [col[i].item() if hasattr(col[i], "item") else col[i] for col in columns]
            payload = struct.pack("<H", msg_id) + packer.pack(*row)
            data_items.append((int(stamps[i]), _message(MSG_TYPE_DATA, payload)))

    data_items.sort(key=lambda item: item[0])
    for _, buffer in data_items:
        out += buffer

    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(out)


def write_synthetic_ulog(
    path: str,
    skip_topics: Sequence[str] = (),
    fast_rate_hz: float = FAST_RATE_HZ,
    slow_rate_hz: float = SLOW_RATE_HZ,
    noise_scale: float = 1.0,
    seed: int = 0,
) -> None:
    """Write the scripted profile to ``path``, omitting any ``skip_topics``."""
    unknown = sorted(set(skip_topics) - set(ALL_TOPICS))
    if unknown:
        raise ValueError(f"unknown topic(s) to skip: {', '.join(unknown)}")
    topics = build_profile(fast_rate_hz, slow_rate_hz, noise_scale, seed)
    for name in skip_topics:
        topics.pop(name, None)
    write_ulog(path, topics, TOPIC_FORMATS, LOG_START_TIMESTAMP_US)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, help="output .ulg path")
    parser.add_argument(
        "--skip-topic",
        action="append",
        default=[],
        choices=ALL_TOPICS,
        help="omit a topic from the log (repeatable)",
    )
    parser.add_argument("--fast-rate-hz", type=float, default=FAST_RATE_HZ, help="rate of acceleration/attitude topics")
    parser.add_argument("--slow-rate-hz", type=float, default=SLOW_RATE_HZ, help="rate of airspeed/position/air data topics")
    parser.add_argument("--noise-scale", type=float, default=1.0, help="multiplier on sensor noise (0 disables)")
    parser.add_argument("--seed", type=int, default=0, help="random seed for sensor noise")
    args = parser.parse_args(argv)

    if args.fast_rate_hz <= 0 or args.slow_rate_hz <= 0 or args.noise_scale < 0:
        parser.error("rates must be positive and noise scale non-negative")

    write_synthetic_ulog(args.out, args.skip_topic, args.fast_rate_hz, args.slow_rate_hz, args.noise_scale, args.seed)
    print(f"wrote {args.out}: {PROFILE_DURATION_S:.0f} s profile, topics: "
          f"{', '.join(t for t in ALL_TOPICS if t not in args.skip_topic)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
