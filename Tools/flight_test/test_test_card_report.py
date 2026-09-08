import csv
import math
import os
import re
import subprocess
import sys

import pytest

import make_synthetic_ulog as synth
import test_card_report as tcr

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLE_CARD = os.path.join(HERE, "example_test_card.csv")

# Profile segments (seconds from log start) mirrored from make_synthetic_ulog.PROFILE
SEGMENTS = {seg.name: seg for seg in synth.PROFILE}
CLIMB = SEGMENTS["climb"]
LEVEL_ACCEL = SEGMENTS["level_accel"]
WIND_UP_TURN = SEGMENTS["wind_up_turn"]
ROLL_OUT = SEGMENTS["roll_out"]
DESCENT = SEGMENTS["descent"]
# The test card treats wind-up turn plus roll-out as one test point.
TURN_WINDOW = (WIND_UP_TURN.start_s, ROLL_OUT.end_s)

NZ_LIMIT_G = 2.5
# Nz = 1/cos(bank): the limit is crossed when bank passes acos(1/limit) on the
# way up (linear ramp in wind_up_turn) and again on the way down (roll_out).
BANK_AT_LIMIT_DEG = math.degrees(math.acos(1.0 / NZ_LIMIT_G))
EXPECTED_FIRST_EXCEEDANCE_S = WIND_UP_TURN.start_s + (WIND_UP_TURN.end_s - WIND_UP_TURN.start_s) * (
    BANK_AT_LIMIT_DEG / WIND_UP_TURN.bank_end_deg
)
EXPECTED_LAST_EXCEEDANCE_S = ROLL_OUT.start_s + (ROLL_OUT.end_s - ROLL_OUT.start_s) * (
    1.0 - BANK_AT_LIMIT_DEG / ROLL_OUT.bank_start_deg
)
EXPECTED_NZ_PEAK_G = 1.0 / math.cos(math.radians(WIND_UP_TURN.bank_end_deg))


@pytest.fixture(scope="session")
def synthetic_ulog(tmp_path_factory):
    path = str(tmp_path_factory.mktemp("ulog") / "synthetic.ulg")
    synth.write_synthetic_ulog(path)
    return path


@pytest.fixture(scope="session")
def flight_log(synthetic_ulog):
    return tcr.FlightLog.from_file(synthetic_ulog)


@pytest.fixture(scope="session")
def no_accel_ulog(tmp_path_factory):
    path = str(tmp_path_factory.mktemp("ulog") / "no_accel.ulg")
    synth.write_synthetic_ulog(path, skip_topics=["vehicle_acceleration"])
    return path


def write_card(path, rows, header=tcr.TEST_CARD_COLUMNS):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)
    return str(path)


# --------------------------------------------------------------------------- test card parsing


def test_parse_test_card_with_blank_limits():
    points = tcr.read_test_card(EXAMPLE_CARD)
    by_id = {tp.test_point_id: tp for tp in points}
    assert list(by_id) == ["TP-01", "TP-02", "TP-03", "TP-04", "TP-05", "TP-06"]

    tp03 = by_id["TP-03"]
    assert (tp03.start_s, tp03.end_s) == (LEVEL_ACCEL.start_s, LEVEL_ACCEL.end_s)
    assert tp03.nz_max_g == 1.5
    assert tp03.airspeed_max_mps is None
    assert tp03.limits() == {"nz_max_g": 1.5, "alt_min_m": 1050.0, "alt_max_m": 1150.0}

    assert by_id["TP-06"].limits() == {}


def test_parse_test_card_tolerates_whitespace(tmp_path):
    path = write_card(tmp_path / "card.csv", [["TP-1", " Trim ", " 0 ", "10", " 1.5 ", "", " ", ""]])
    (tp,) = tcr.read_test_card(path)
    assert tp.description == "Trim"
    assert tp.limits() == {"nz_max_g": 1.5}


@pytest.mark.parametrize(
    "row, message",
    [
        (["bad id!", "x", "0", "10", "", "", "", ""], "test_point_id"),
        (["TP-1", "x", "10", "10", "", "", "", ""], "start_s < end_s"),
        (["TP-1", "x", "-1", "10", "", "", "", ""], "start_s < end_s"),
        (["TP-1", "x", "0", "10", "abc", "", "", ""], "nz_max_g must be numeric"),
        (["TP-1", "x", "0", "10", "", "", "1000", "900"], "alt_min_m must be below alt_max_m"),
        (["TP-1", "x", "0", "10", "0", "", "", ""], "nz_max_g must be positive"),
        (["TP-1", "x", "", "10", "", "", "", ""], "required"),
    ],
)
def test_parse_test_card_rejects_invalid_rows(tmp_path, row, message):
    path = write_card(tmp_path / "card.csv", [row])
    with pytest.raises(tcr.TestCardError, match=message):
        tcr.read_test_card(path)


def test_parse_test_card_rejects_duplicate_ids_and_bad_header(tmp_path):
    path = write_card(
        tmp_path / "dup.csv",
        [["TP-1", "a", "0", "10", "", "", "", ""], ["TP-1", "b", "10", "20", "", "", "", ""]],
    )
    with pytest.raises(tcr.TestCardError, match="duplicate"):
        tcr.read_test_card(path)

    path = write_card(tmp_path / "hdr.csv", [["TP-1", "a", "0", "10"]], header=["test_point_id", "description", "start_s"])
    with pytest.raises(tcr.TestCardError, match="missing required column"):
        tcr.read_test_card(path)

    path = write_card(tmp_path / "unknown.csv", [["TP-1", "a", "0", "10", "1"]], header=list(tcr.TEST_CARD_COLUMNS[:4]) + ["mach_max"])
    with pytest.raises(tcr.TestCardError, match="unknown column"):
        tcr.read_test_card(path)


# --------------------------------------------------------------------------- synthetic log


def test_synthetic_ulog_round_trips_through_pyulog(flight_log):
    assert flight_log.start_timestamp_us == synth.LOG_START_TIMESTAMP_US
    assert flight_log.duration_s == pytest.approx(synth.PROFILE_DURATION_S, abs=0.2)
    assert all(flight_log.channels[key] is not None for key in tcr.CHANNEL_LABELS)
    assert flight_log.source_notes["nz_g"] == "vehicle_acceleration.xyz[2]"
    assert flight_log.source_notes["altitude_m"] == "vehicle_local_position.ref_alt - z"
    assert flight_log.source_notes["sink_rate_mps"] == "vehicle_local_position.vz"


def test_synthetic_profile_is_continuous():
    for previous, following in zip(synth.PROFILE, synth.PROFILE[1:]):
        assert previous.end_s == following.start_s
        assert previous.alt_end_m == following.alt_start_m
        assert previous.airspeed_end_mps == following.airspeed_start_mps
        assert previous.bank_end_deg == following.bank_start_deg


# --------------------------------------------------------------------------- statistics


def test_level_accel_statistics(flight_log):
    tp = tcr.TestPoint("TP-03", "level accel", LEVEL_ACCEL.start_s, LEVEL_ACCEL.end_s)
    r = tcr.reduce_test_point(tp, flight_log)

    assert r.status == tcr.STATUS_COMPLETE
    assert r.duration_flown_s == pytest.approx(LEVEL_ACCEL.end_s - LEVEL_ACCEL.start_s, abs=0.1)
    assert r.nz_peak_g == pytest.approx(1.0, abs=0.05)
    assert r.nz_mean_g == pytest.approx(1.0, abs=0.01)
    assert r.airspeed_min_mps == pytest.approx(LEVEL_ACCEL.airspeed_start_mps, abs=0.5)
    assert r.airspeed_max_mps == pytest.approx(LEVEL_ACCEL.airspeed_end_mps, abs=0.5)
    assert r.alt_min_flown_m == pytest.approx(LEVEL_ACCEL.alt_start_m, abs=2.0)
    assert r.alt_max_flown_m == pytest.approx(LEVEL_ACCEL.alt_end_m, abs=2.0)
    assert r.bank_max_deg == pytest.approx(0.0, abs=1.0)
    assert r.sink_rate_max_mps == pytest.approx(0.0, abs=0.5)
    assert r.exceedances == []
    assert r.notes == []


def test_climb_and_descent_statistics(flight_log):
    climb = tcr.reduce_test_point(tcr.TestPoint("TP-02", "climb", CLIMB.start_s, CLIMB.end_s), flight_log)
    assert climb.alt_min_flown_m == pytest.approx(CLIMB.alt_start_m, abs=2.0)
    assert climb.alt_max_flown_m == pytest.approx(CLIMB.alt_end_m, abs=2.0)
    assert climb.sink_rate_max_mps < 0.5

    descent = tcr.reduce_test_point(tcr.TestPoint("TP-05", "descent", DESCENT.start_s, DESCENT.end_s), flight_log)
    expected_sink = (DESCENT.alt_start_m - DESCENT.alt_end_m) / (DESCENT.end_s - DESCENT.start_s)
    assert descent.sink_rate_max_mps == pytest.approx(expected_sink, abs=0.5)
    assert descent.alt_min_flown_m == pytest.approx(DESCENT.alt_end_m, abs=2.0)
    assert descent.alt_max_flown_m == pytest.approx(DESCENT.alt_start_m, abs=2.0)


def test_duration_flown_is_clipped_to_logged_data(flight_log):
    tp = tcr.TestPoint("TP-X", "runs past end of log", 270.0, 400.0)
    r = tcr.reduce_test_point(tp, flight_log)
    assert r.status == tcr.STATUS_COMPLETE
    assert r.duration_flown_s == pytest.approx(synth.PROFILE_DURATION_S - 270.0, abs=0.2)


# --------------------------------------------------------------------------- exceedances


def test_wind_up_turn_busts_nz_limit(flight_log):
    tp = tcr.TestPoint(
        "TP-04", "wind-up turn", *TURN_WINDOW,
        nz_max_g=NZ_LIMIT_G, airspeed_max_mps=50.0, alt_min_m=1050.0, alt_max_m=1150.0,
    )
    r = tcr.reduce_test_point(tp, flight_log)

    assert r.status == tcr.STATUS_EXCEEDANCE
    assert r.bank_max_deg == pytest.approx(WIND_UP_TURN.bank_end_deg, abs=0.5)
    assert r.nz_peak_g == pytest.approx(EXPECTED_NZ_PEAK_G, abs=0.02)

    assert [e.limit for e in r.exceedances] == ["nz_max_g"]
    (exceedance,) = r.exceedances
    assert exceedance.limit_value == NZ_LIMIT_G
    assert exceedance.peak_value == pytest.approx(EXPECTED_NZ_PEAK_G, abs=0.02)
    assert exceedance.first_exceedance_s == pytest.approx(EXPECTED_FIRST_EXCEEDANCE_S, abs=0.1)
    assert exceedance.seconds_outside == pytest.approx(
        EXPECTED_LAST_EXCEEDANCE_S - EXPECTED_FIRST_EXCEEDANCE_S, abs=0.1
    )


def test_min_and_max_altitude_limits(flight_log):
    tp = tcr.TestPoint("TP-05", "descent", DESCENT.start_s, DESCENT.end_s, alt_min_m=800.0, alt_max_m=1000.0)
    r = tcr.reduce_test_point(tp, flight_log)

    assert r.status == tcr.STATUS_EXCEEDANCE
    by_limit = {e.limit: e for e in r.exceedances}
    assert set(by_limit) == {"alt_min_m", "alt_max_m"}
    # 1100 -> 500 m over 60 s: above 1000 m for the first 10 s, below 800 m for the last 30 s
    assert by_limit["alt_max_m"].peak_value == pytest.approx(DESCENT.alt_start_m, abs=2.0)
    assert by_limit["alt_max_m"].first_exceedance_s == pytest.approx(DESCENT.start_s, abs=0.2)
    assert by_limit["alt_max_m"].seconds_outside == pytest.approx(10.0, abs=0.5)
    assert by_limit["alt_min_m"].peak_value == pytest.approx(DESCENT.alt_end_m, abs=2.0)
    assert by_limit["alt_min_m"].first_exceedance_s == pytest.approx(DESCENT.start_s + 30.0, abs=0.5)
    assert by_limit["alt_min_m"].seconds_outside == pytest.approx(30.0, abs=0.5)


def test_within_limits_is_complete(flight_log):
    tp = tcr.TestPoint(
        "TP-04", "wind-up turn", *TURN_WINDOW,
        nz_max_g=3.5, airspeed_max_mps=50.0, alt_min_m=1050.0, alt_max_m=1150.0,
    )
    r = tcr.reduce_test_point(tp, flight_log)
    assert r.status == tcr.STATUS_COMPLETE
    assert r.exceedances == []


# --------------------------------------------------------------------------- NO DATA paths


def test_missing_topic_marks_limit_as_no_data(no_accel_ulog):
    log = tcr.FlightLog.from_file(no_accel_ulog)
    assert log.channels["nz_g"] is None
    assert log.source_notes["nz_g"] == "vehicle_acceleration not logged"

    with_limit = tcr.reduce_test_point(tcr.TestPoint("TP-04", "turn", *TURN_WINDOW, nz_max_g=NZ_LIMIT_G), log)
    assert with_limit.status == tcr.STATUS_NO_DATA
    assert with_limit.nz_peak_g is None
    assert with_limit.bank_max_deg == pytest.approx(WIND_UP_TURN.bank_end_deg, abs=0.5)
    assert any("vehicle_acceleration not logged" in note for note in with_limit.notes)
    assert any(note.startswith("limit nz_max_g=2.5 not evaluated") for note in with_limit.notes)

    without_limit = tcr.reduce_test_point(
        tcr.TestPoint("TP-05", "descent", DESCENT.start_s, DESCENT.end_s, alt_min_m=450.0), log
    )
    assert without_limit.status == tcr.STATUS_COMPLETE
    assert without_limit.nz_peak_g is None
    assert without_limit.notes == ["Nz [g]: vehicle_acceleration not logged"]


def test_window_outside_log_is_no_data(flight_log):
    r = tcr.reduce_test_point(tcr.TestPoint("TP-06", "not flown", 300.0, 330.0), flight_log)
    assert r.status == tcr.STATUS_NO_DATA
    assert r.duration_flown_s is None
    assert r.nz_peak_g is None
    assert r.exceedances == []
    assert len(r.notes) == len(tcr.CHANNEL_LABELS)
    assert all(note.endswith("no samples in window") for note in r.notes)


def test_altitude_falls_back_to_baro_without_local_position(tmp_path):
    path = str(tmp_path / "no_lpos.ulg")
    synth.write_synthetic_ulog(path, skip_topics=["vehicle_local_position"])
    log = tcr.FlightLog.from_file(path)
    assert log.source_notes["altitude_m"] == "vehicle_air_data.baro_alt_meter"
    assert log.source_notes["sink_rate_mps"].startswith("-d/dt vehicle_air_data.baro_alt_meter")

    r = tcr.reduce_test_point(tcr.TestPoint("TP-05", "descent", DESCENT.start_s + 5, DESCENT.end_s - 5), log)
    assert r.status == tcr.STATUS_COMPLETE
    assert r.sink_rate_max_mps == pytest.approx(10.0, abs=1.5)
    assert r.alt_max_flown_m == pytest.approx(1050.0, abs=5.0)


# --------------------------------------------------------------------------- outputs


def test_cli_writes_markdown_and_csv(synthetic_ulog, tmp_path, capsys):
    out_dir = tmp_path / "out"
    assert tcr.main([synthetic_ulog, EXAMPLE_CARD, "--out", str(out_dir)]) == 0

    stdout = capsys.readouterr().out
    assert stdout.count("\n") == 1
    assert stdout.startswith("6 test points: 4 complete, 1 with exceedance, 1 no data")

    with open(out_dir / "report.csv", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert list(rows[0].keys()) == list(tcr.REPORT_CSV_COLUMNS)
    by_id = {row["test_point_id"]: row for row in rows}
    assert len(by_id) == 6
    assert by_id["TP-03"]["status"] == tcr.STATUS_COMPLETE
    assert by_id["TP-04"]["status"] == tcr.STATUS_EXCEEDANCE
    assert by_id["TP-04"]["exceedance_count"] == "1"
    assert by_id["TP-04"]["exceedances"].startswith("nz_max_g=2.5 peak=2.9")
    assert by_id["TP-06"]["status"] == tcr.STATUS_NO_DATA
    assert by_id["TP-06"]["nz_peak_g"] == ""
    assert float(by_id["TP-05"]["sink_rate_max_mps"]) == pytest.approx(10.0, abs=0.5)

    markdown = (out_dir / "report.md").read_text()
    assert markdown.startswith("# Flight test card report")
    assert "## Sortie summary" in markdown
    assert "## Test point details" in markdown
    for test_point_id in by_id:
        assert f"### {test_point_id} - " in markdown
    assert "| TP-04 | Wind-up turn to 70 deg bank | 140-200 |" in markdown
    assert "| nz_max_g | 2.5 | 2.9" in markdown
    assert "**COMPLETE WITH EXCEEDANCE**" in markdown
    assert "no samples in window" in markdown


def test_markdown_escapes_pipes(flight_log, tmp_path, synthetic_ulog):
    tp = tcr.TestPoint("TP-1", "a | b", LEVEL_ACCEL.start_s, LEVEL_ACCEL.end_s)
    markdown = tcr.render_markdown(tcr.reduce_test_card([tp], flight_log), flight_log, synthetic_ulog, "card.csv")
    assert "| TP-1 | a \\| b |" in markdown


def test_cli_end_to_end_subprocess(tmp_path):
    ulog = tmp_path / "flight.ulg"
    generate = subprocess.run(
        [sys.executable, os.path.join(HERE, "make_synthetic_ulog.py"), "--out", str(ulog)],
        capture_output=True, text=True, check=True,
    )
    assert generate.stdout.startswith("wrote ")

    report = subprocess.run(
        [sys.executable, os.path.join(HERE, "test_card_report.py"), str(ulog), EXAMPLE_CARD, "--out", str(tmp_path / "out")],
        capture_output=True, text=True, check=True,
    )
    assert report.stdout.startswith("6 test points:")
    assert (tmp_path / "out" / "report.md").is_file()
    assert (tmp_path / "out" / "report.csv").is_file()


def test_cli_rejects_invalid_card(synthetic_ulog, tmp_path, capsys):
    path = write_card(tmp_path / "bad.csv", [["TP-1", "x", "10", "5", "", "", "", ""]])
    assert tcr.main([synthetic_ulog, path, "--out", str(tmp_path / "out")]) == 2
    assert "invalid test card" in capsys.readouterr().err


# --------------------------------------------------------------------------- docs stay in sync


def _readme_table_columns(section_heading):
    readme = open(os.path.join(HERE, "README.md"), encoding="utf-8").read()
    section = readme.split(section_heading, 1)[1].split("\n## ", 1)[0]
    return set(re.findall(r"^\| `([a-z_]+)` \|", section, flags=re.MULTILINE))


def test_readme_documents_test_card_and_report_columns():
    assert _readme_table_columns("## Test card CSV schema") == set(tcr.TEST_CARD_COLUMNS)
    assert _readme_table_columns("## report.csv columns") == set(tcr.REPORT_CSV_COLUMNS)


def test_example_card_covers_every_profile_segment_with_a_bust():
    points = tcr.read_test_card(EXAMPLE_CARD)
    windows = {(tp.start_s, tp.end_s) for tp in points}
    for seg in synth.PROFILE:
        if seg.name not in ("wind_up_turn", "roll_out", "trim_out"):
            assert (seg.start_s, seg.end_s) in windows, seg.name
    assert TURN_WINDOW in windows
    turn = next(tp for tp in points if (tp.start_s, tp.end_s) == TURN_WINDOW)
    assert turn.nz_max_g == NZ_LIMIT_G < EXPECTED_NZ_PEAK_G
