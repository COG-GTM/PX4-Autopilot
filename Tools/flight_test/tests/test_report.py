"""pytest coverage for test_card_report.py and synthetic_ulog.py."""

import csv
import os

import numpy as np
import pytest
from pyulog import ULog

import synthetic_ulog as synth
import test_card_report as tcr

TOOL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_CARD = os.path.join(TOOL_DIR, "example_test_card.csv")

# A small deterministic profile: 1 g cruise, a pull-up peaking at 3 g, a
# 30 deg bank, then a dive that busts an altitude floor.
PROFILE = (
    synth.Keyframe(0.0, 1.0, 20.0, 500.0, 0.0),
    synth.Keyframe(10.0, 1.0, 20.0, 500.0, 0.0),
    synth.Keyframe(12.0, 3.0, 24.0, 500.0, 0.0),
    synth.Keyframe(14.0, 1.0, 24.0, 500.0, 0.0),
    synth.Keyframe(20.0, 1.0, 24.0, 500.0, 30.0),
    synth.Keyframe(25.0, 1.0, 24.0, 500.0, 30.0),
    synth.Keyframe(30.0, 1.0, 24.0, 500.0, 0.0),
    synth.Keyframe(40.0, 1.0, 30.0, 400.0, 0.0),
    synth.Keyframe(50.0, 1.0, 20.0, 400.0, 0.0),
)


def write_card(path, rows):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(tcr.TEST_CARD_COLUMNS)
        writer.writerows(rows)
    return str(path)


@pytest.fixture(scope="module")
def full_ulog(tmp_path_factory):
    path = tmp_path_factory.mktemp("ulog") / "full.ulg"
    synth.write_synthetic_ulog(str(path), keyframes=PROFILE, boot_offset_s=12.5)
    return str(path)


@pytest.fixture(scope="module")
def full_flight(full_ulog):
    return tcr.load_flight_data(full_ulog)


# --- synthetic ULog writer -------------------------------------------------

def test_synthetic_ulog_is_parseable_by_pyulog(full_ulog):
    ulog = ULog(full_ulog)
    names = {d.name for d in ulog.data_list}
    assert names == set(synth.ALL_TOPICS)
    assert ulog.start_timestamp == 12_500_000
    accel = ulog.get_dataset("vehicle_acceleration")
    assert accel.data["timestamp"][0] == 12_500_000
    assert accel.data["xyz[2]"][0] == pytest.approx(-synth.G0, rel=1e-6)
    assert ulog.msg_info_dict["sys_name"] == "PX4"


def test_synthetic_ulog_omits_requested_topics(tmp_path):
    path = tmp_path / "partial.ulg"
    synth.write_synthetic_ulog(str(path), keyframes=PROFILE,
                               topics=[synth.TOPIC_ACCEL, synth.TOPIC_AIR_DATA])
    names = {d.name for d in ULog(str(path)).data_list}
    assert names == {synth.TOPIC_ACCEL, synth.TOPIC_AIR_DATA}


def test_synthetic_ulog_rejects_bad_profiles():
    with pytest.raises(ValueError):
        synth.build_synthetic_ulog(keyframes=PROFILE[:1])
    with pytest.raises(ValueError):
        synth.build_synthetic_ulog(keyframes=(synth.Keyframe(5.0), synth.Keyframe(5.0)))
    with pytest.raises(ValueError):
        synth.build_synthetic_ulog(keyframes=PROFILE, topics=["not_a_topic"])


def test_synthetic_ulog_cli(tmp_path, capsys):
    out = tmp_path / "cli.ulg"
    assert synth.main([str(out), "--without", "airspeed_validated"]) == 0
    assert "airspeed_validated" not in capsys.readouterr().out
    assert "airspeed_validated" not in {d.name for d in ULog(str(out)).data_list}


# --- channel extraction -----------------------------------------------------

def test_load_flight_data_extracts_expected_channels(full_flight):
    fd = full_flight
    assert all(fd.topics_present[t] for t in tcr.ULOG_TOPICS)
    assert fd.nz_g.t[0] == pytest.approx(0.0)
    assert fd.nz_g.v[0] == pytest.approx(1.0, abs=1e-5)
    assert np.max(fd.nz_g.v) == pytest.approx(3.0, abs=1e-5)
    assert np.max(fd.cas_mps.v) == pytest.approx(30.0, abs=1e-4)
    assert fd.alt_source.startswith("vehicle_local_position")
    assert np.min(fd.alt_m.v) == pytest.approx(400.0, abs=1e-2)
    assert np.max(fd.bank_deg.v) == pytest.approx(30.0, abs=1e-3)
    # 100 m descent in 10 s
    assert np.max(fd.vz_down_mps.v) == pytest.approx(10.0, abs=0.05)


def test_time_origin_boot_shifts_channels(full_ulog):
    fd = tcr.load_flight_data(full_ulog, time_origin="boot")
    assert fd.nz_g.t[0] == pytest.approx(12.5)
    with pytest.raises(ValueError):
        tcr.load_flight_data(full_ulog, time_origin="takeoff")


def test_quaternion_to_roll_deg_matches_pure_roll():
    roll = np.radians([-60.0, 0.0, 45.0])
    q = synth.roll_to_quaternion(roll)
    assert tcr.quaternion_to_roll_deg(q) == pytest.approx([-60.0, 0.0, 45.0])


def test_local_position_without_global_reference_uses_negative_z(tmp_path):
    path = tmp_path / "local.ulg"
    synth.write_synthetic_ulog(str(path), keyframes=PROFILE, z_global=False)
    fd = tcr.load_flight_data(str(path))
    assert "relative to local origin" in fd.alt_source
    assert fd.alt_m.v[0] == pytest.approx(500.0, abs=1e-3)


def test_air_data_fallback_when_local_position_missing(tmp_path):
    path = tmp_path / "baro.ulg"
    topics = [t for t in synth.ALL_TOPICS if t != synth.TOPIC_LOCAL_POS]
    synth.write_synthetic_ulog(str(path), keyframes=PROFILE, topics=topics)
    fd = tcr.load_flight_data(str(path))
    assert fd.topics_present[tcr.TOPIC_LOCAL_POS] is False
    assert fd.alt_source.startswith("vehicle_air_data")
    assert "d/dt" in fd.vz_source
    assert np.min(fd.alt_m.v) == pytest.approx(400.0, abs=1e-2)
    assert np.max(fd.vz_down_mps.v) == pytest.approx(10.0, abs=0.1)


def test_channel_drops_nan_samples():
    ch = tcr.Channel("x", np.array([0.0, 1.0, 2.0]), np.array([1.0, np.nan, 3.0]))
    assert len(ch) == 2
    assert list(ch.v) == [1.0, 3.0]
    assert list(ch.sample_durations()) == [2.0, 2.0]


# --- reduction ---------------------------------------------------------------

def test_complete_test_point_metrics(full_flight):
    tp = tcr.TestPoint("TP-A", "cruise", 0.0, 10.0, nz_max_g=1.5, airspeed_max_mps=25.0,
                       alt_min_m=450.0, alt_max_m=550.0)
    r = tcr.reduce_test_point(full_flight, tp)
    assert r.status == tcr.STATUS_COMPLETE
    assert r.duration_flown_s == pytest.approx(10.0)
    assert r.nz_peak_g == pytest.approx(1.0, abs=1e-5)
    assert r.nz_mean_g == pytest.approx(1.0, abs=1e-5)
    assert r.airspeed_min_mps == pytest.approx(20.0, abs=1e-4)
    assert r.airspeed_max_mps == pytest.approx(20.0, abs=1e-4)
    assert r.alt_min_m == pytest.approx(500.0, abs=1e-3)
    assert r.alt_max_m == pytest.approx(500.0, abs=1e-3)
    assert r.bank_max_deg == pytest.approx(0.0, abs=1e-6)
    assert r.sink_rate_max_mps == pytest.approx(0.0, abs=1e-6)
    assert r.exceedances == []
    assert r.notes == []


def test_nz_exceedance_reports_limit_peak_duration_and_first_time(full_flight):
    tp = tcr.TestPoint("TP-B", "pull-up", 10.0, 16.0, nz_max_g=2.0)
    r = tcr.reduce_test_point(full_flight, tp)
    assert r.status == tcr.STATUS_EXCEEDANCE
    assert len(r.exceedances) == 1
    exc = r.exceedances[0]
    assert exc.parameter == "Nz [g]"
    assert exc.kind == "max"
    assert exc.limit == 2.0
    assert exc.peak == pytest.approx(3.0, abs=1e-5)
    # Nz ramps 1 -> 3 g over 10..12 s and back over 12..14 s: above 2 g from 11 s to 13 s.
    assert exc.first_time_s == pytest.approx(11.0, abs=0.01)
    assert exc.seconds_outside == pytest.approx(2.0, abs=0.02)


def test_altitude_floor_and_airspeed_exceedances(full_flight):
    tp = tcr.TestPoint("TP-C", "dive", 30.0, 50.0, airspeed_max_mps=28.0, alt_min_m=450.0, alt_max_m=600.0)
    r = tcr.reduce_test_point(full_flight, tp)
    assert r.status == tcr.STATUS_EXCEEDANCE
    by_param = {(e.parameter, e.kind): e for e in r.exceedances}
    assert set(by_param) == {("CAS [m/s]", "max"), ("altitude [m]", "min")}
    cas = by_param[("CAS [m/s]", "max")]
    # CAS ramps 24 -> 30 over 30..40 s (crosses 28 at 36.67 s) and back 30 -> 20 over 40..50 s
    # (crosses 28 at 42 s), sampled at 10 Hz.
    assert cas.peak == pytest.approx(30.0, abs=1e-4)
    assert cas.first_time_s == pytest.approx(36.7, abs=0.11)
    assert cas.seconds_outside == pytest.approx(5.33, abs=0.15)
    alt = by_param[("altitude [m]", "min")]
    # altitude descends 500 -> 400 over 30..40 s, below 450 m from 35 s to the end of the window.
    assert alt.peak == pytest.approx(400.0, abs=1e-2)
    assert alt.first_time_s == pytest.approx(35.0, abs=0.03)
    assert alt.seconds_outside == pytest.approx(15.0, abs=0.05)


def test_no_data_when_window_is_outside_the_log(full_flight):
    tp = tcr.TestPoint("TP-D", "not flown", 100.0, 120.0, nz_max_g=2.0)
    r = tcr.reduce_test_point(full_flight, tp)
    assert r.status == tcr.STATUS_NO_DATA
    assert r.duration_flown_s == 0.0
    assert r.nz_peak_g is None
    assert r.metric_text("nz_peak_g") == tcr.NO_DATA
    assert r.exceedances == []
    assert len(r.notes) == 1 and "no samples" in r.notes[0]


def test_partially_flown_window_reports_duration_flown(full_flight):
    tp = tcr.TestPoint("TP-E", "runs off the end", 45.0, 65.0)
    r = tcr.reduce_test_point(full_flight, tp)
    assert r.status == tcr.STATUS_COMPLETE
    assert r.duration_flown_s == pytest.approx(5.0, abs=0.02)


def test_missing_topic_is_reported_as_not_logged(tmp_path):
    path = tmp_path / "no_airspeed.ulg"
    topics = [t for t in synth.ALL_TOPICS if t != synth.TOPIC_AIRSPEED]
    synth.write_synthetic_ulog(str(path), keyframes=PROFILE, topics=topics)
    fd = tcr.load_flight_data(str(path))
    tp = tcr.TestPoint("TP-F", "no pitot", 0.0, 10.0, airspeed_max_mps=25.0, nz_max_g=1.5)
    r = tcr.reduce_test_point(fd, tp)
    assert r.status == tcr.STATUS_COMPLETE
    assert r.airspeed_max_mps is None
    assert r.metric_text("airspeed_max_mps") == tcr.NOT_LOGGED
    assert r.metric_text("nz_peak_g") == "1.00"
    assert any("airspeed_max_mps limit 25.00 not evaluated (not logged)" == n for n in r.notes)


def test_all_topics_missing_gives_no_data(tmp_path):
    path = tmp_path / "empty.ulg"
    synth.write_synthetic_ulog(str(path), keyframes=PROFILE, topics=[])
    fd = tcr.load_flight_data(str(path))
    assert fd.data_span() is None
    r = tcr.reduce_test_point(fd, tcr.TestPoint("TP-G", "empty", 0.0, 10.0))
    assert r.status == tcr.STATUS_NO_DATA
    assert all(v == tcr.NOT_LOGGED for v in r.availability.values())


def test_find_exceedance_edge_cases():
    ch = tcr.Channel("x", np.array([0.0, 1.0, 2.0, 3.0]), np.array([1.0, 2.0, 3.0, 1.0]))
    assert tcr.find_exceedance(ch, "x", "max", 3.0) is None
    exc = tcr.find_exceedance(ch, "x", "max", 1.5)
    assert (exc.first_time_s, exc.seconds_outside, exc.peak) == (1.0, 2.0, 3.0)
    exc = tcr.find_exceedance(ch, "x", "min", 1.5)
    assert (exc.first_time_s, exc.seconds_outside, exc.peak) == (0.0, 2.0, 1.0)
    assert tcr.find_exceedance(ch.window(10.0, 20.0), "x", "max", 0.0) is None
    with pytest.raises(ValueError):
        tcr.find_exceedance(ch, "x", "between", 1.0)


# --- test card parsing --------------------------------------------------------

def test_example_test_card_loads_and_matches_documented_columns():
    points = tcr.load_test_card(EXAMPLE_CARD)
    with open(EXAMPLE_CARD, newline="") as handle:
        header = next(csv.reader(handle))
    assert tuple(header) == tcr.TEST_CARD_COLUMNS
    assert [p.test_point_id for p in points] == ["TP-01", "TP-02", "TP-03", "TP-04", "TP-05", "TP-06"]
    assert points[5].nz_max_g is None and points[5].alt_max_m is None
    assert points[1].nz_max_g == 2.5


def test_readme_documents_every_test_card_and_report_column():
    with open(os.path.join(TOOL_DIR, "README.md")) as handle:
        readme = handle.read()
    for column in tcr.TEST_CARD_COLUMNS + tcr.REPORT_CSV_COLUMNS:
        assert "`%s`" % column in readme, column
    for status in (tcr.STATUS_COMPLETE, tcr.STATUS_EXCEEDANCE, tcr.STATUS_NO_DATA):
        assert status in readme


def test_test_card_accepts_blank_lines_and_bom(tmp_path):
    path = tmp_path / "card.csv"
    path.write_bytes(b"\xef\xbb\xbf" + ",".join(tcr.TEST_CARD_COLUMNS).encode() +
                     b"\nTP-1,desc,0,10,,,,\n,,,,,,,\nTP-2,desc two,10,20,2,30,,900\n")
    points = tcr.load_test_card(str(path))
    assert [p.test_point_id for p in points] == ["TP-1", "TP-2"]
    assert points[1].airspeed_max_mps == 30.0 and points[1].alt_min_m is None


@pytest.mark.parametrize("rows,message", [
    ([["bad id!", "d", 0, 10, "", "", "", ""]], "test_point_id"),
    ([["TP-1", "d", 0, 10, "", "", "", ""], ["TP-1", "d", 10, 20, "", "", "", ""]], "duplicate"),
    ([["TP-1", "d", 10, 10, "", "", "", ""]], "end_s"),
    ([["TP-1", "d", -1, 10, "", "", "", ""]], "within"),
    ([["TP-1", "d", "abc", 10, "", "", "", ""]], "numeric"),
    ([["TP-1", "d", 0, 10, "nan", "", "", ""]], "finite"),
    ([["TP-1", "d", 0, 10, "-2", "", "", ""]], "positive"),
    ([["TP-1", "d", 0, 10, "", "", "600", "500"]], "alt_min_m must be below"),
    ([["TP-1", "x" * 300, 0, 10, "", "", "", ""]], "description"),
    ([], "no test points"),
])
def test_test_card_validation_errors(tmp_path, rows, message):
    path = write_card(tmp_path / "card.csv", rows)
    with pytest.raises(tcr.TestCardError, match=message):
        tcr.load_test_card(path)


def test_test_card_header_validation(tmp_path):
    path = tmp_path / "card.csv"
    path.write_text("test_point_id,description,start_s\nTP-1,d,0\n")
    with pytest.raises(tcr.TestCardError, match="missing column"):
        tcr.load_test_card(str(path))
    path.write_text(",".join(tcr.TEST_CARD_COLUMNS + ("pilot",)) + "\nTP-1,d,0,10,,,,,Bob\n")
    with pytest.raises(tcr.TestCardError, match="unexpected column"):
        tcr.load_test_card(str(path))
    path.write_text("")
    with pytest.raises(tcr.TestCardError, match="empty"):
        tcr.load_test_card(str(path))


# --- end-to-end CLI -------------------------------------------------------------

def test_cli_writes_reports_and_summary(tmp_path, capsys):
    ulog = tmp_path / "demo.ulg"
    synth.write_synthetic_ulog(str(ulog))
    out = tmp_path / "out"
    rc = tcr.main([str(ulog), EXAMPLE_CARD, "--out", str(out)])
    assert rc == 0
    summary = capsys.readouterr().out.strip()
    assert summary.startswith("6 test points: 3 COMPLETE, 2 COMPLETE WITH EXCEEDANCE, 1 NO DATA -> ")
    assert summary.count("\n") == 0

    md = (out / "report.md").read_text()
    assert "# Flight Test Card Report" in md
    assert "### TP-02: Symmetric pull-up to 2.5 g" in md
    assert "| Nz [g] (max) | 2.50 | 2.80 |" in md
    assert "### TP-05" in md and "**NO DATA**" in md

    with open(out / "report.csv", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert tuple(rows[0].keys()) == tcr.REPORT_CSV_COLUMNS
    by_id = {r["test_point_id"]: r for r in rows}
    assert by_id["TP-01"]["status"] == tcr.STATUS_COMPLETE
    assert by_id["TP-02"]["status"] == tcr.STATUS_EXCEEDANCE
    assert by_id["TP-02"]["exceedance_count"] == "1"
    assert float(by_id["TP-02"]["nz_peak_g"]) == pytest.approx(2.8, abs=1e-3)
    assert by_id["TP-04"]["exceedance_count"] == "2"
    assert float(by_id["TP-04"]["airspeed_max_mps"]) == pytest.approx(38.0, abs=1e-3)
    assert float(by_id["TP-04"]["alt_min_m"]) == pytest.approx(420.0, abs=1e-2)
    assert by_id["TP-05"]["status"] == tcr.STATUS_NO_DATA
    assert by_id["TP-05"]["nz_peak_g"] == tcr.NO_DATA
    assert by_id["TP-06"]["exceedances"] == ""


def test_cli_reports_not_logged_topics(tmp_path, capsys):
    ulog = tmp_path / "no_airspeed.ulg"
    synth.write_synthetic_ulog(str(ulog), topics=[t for t in synth.ALL_TOPICS if t != synth.TOPIC_AIRSPEED])
    out = tmp_path / "out"
    assert tcr.main([str(ulog), EXAMPLE_CARD, "--out", str(out)]) == 0
    md = (out / "report.md").read_text()
    assert "Topics not logged: `airspeed_validated`" in md
    assert "CAS: min not logged, max not logged" in md
    with open(out / "report.csv", newline="") as handle:
        rows = {r["test_point_id"]: r for r in csv.DictReader(handle)}
    assert rows["TP-01"]["airspeed_max_mps"] == tcr.NOT_LOGGED
    assert "airspeed_max_mps limit 30.00 not evaluated (not logged)" in rows["TP-01"]["notes"]
    # TP-04 still busts its altitude floor even without airspeed data
    assert rows["TP-04"]["status"] == tcr.STATUS_EXCEEDANCE
    assert rows["TP-04"]["exceedance_count"] == "1"


def test_cli_error_paths(tmp_path, capsys):
    ulog = tmp_path / "demo.ulg"
    synth.write_synthetic_ulog(str(ulog))
    assert tcr.main([str(tmp_path / "missing.ulg"), EXAMPLE_CARD, "--out", str(tmp_path)]) == 2
    assert "ULog file not found" in capsys.readouterr().err
    bad_card = write_card(tmp_path / "bad.csv", [["TP-1", "d", 10, 5, "", "", "", ""]])
    assert tcr.main([str(ulog), bad_card, "--out", str(tmp_path)]) == 2
    assert "invalid test card" in capsys.readouterr().err
    not_ulog = tmp_path / "junk.ulg"
    not_ulog.write_bytes(b"this is not a ulog file")
    assert tcr.main([str(not_ulog), EXAMPLE_CARD, "--out", str(tmp_path)]) == 1
    assert "could not process inputs" in capsys.readouterr().err
