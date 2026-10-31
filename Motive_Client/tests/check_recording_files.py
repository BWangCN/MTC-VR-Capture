"""Run the isolated native recorder fixture, then validate actual JSON/CSV files."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def check(executable):
    with tempfile.TemporaryDirectory(prefix="mtc_recording_checks_") as temporary:
        root = Path(temporary)
        subprocess.run([executable, str(root)], check=True)
        takes = sorted((root / "normal").iterdir())
        assert len(takes) == 2, "Retries or stale commands created an extra take"
        first, second = takes
        metadata = json.loads((first / "metadata.json").read_text())
        assert metadata["position_units"] == "m"
        assert metadata["source"]["models"][0]["name"] == "Pico"
        assert metadata["source"]["models"][0]["id"] == 42
        frames = [json.loads(line) for line in (first / "frames.jsonl").read_text().splitlines()]
        assert [f["frame"] for f in frames] == [10, 12, 13]
        f = frames[0]
        assert f["other_markers"] == [[1.25, 0, 0], [0, 2.5, 0]], "Callback data was not deep-copied"
        assert f["marker_sets"][0]["name"] == 'set "A"'
        assert f["marker_sets"][0]["positions"] == f["other_markers"]
        assert f["labeled_markers"][0]["id"] == 123
        assert f["labeled_markers"][0]["position"][0] == 9
        assert f["rigid_bodies"][0]["position"][0] == 4.5
        assert f["skeletons"][0]["rigid_bodies"][0]["position"][0] == 4.5
        assert f["assets"][0]["markers"][0]["id"] == 123
        assert f["force_plates"][0]["channels"] == [[0, 3.25]]
        assert f["devices"][0]["channels"] == [[7.5]]
        assert f["imu"][0]["quaternion_xyzw"][0] == 0.25
        assert f["gpio"][0]["values"] == [0, 65535]
        assert (f["camera_mid_exposure_ticks"], f["camera_data_received_ticks"], f["transmit_ticks"]) == (100, 110, 120)
        assert (f["timecode"], f["timecode_subframe"], f["params"]) == (1234, 3, 7)
        assert (f["precision_timestamp_secs"], f["precision_timestamp_fractional_secs"]) == (123, 456)
        assert f["receive_unix_ns"] == 1000000000 and f["receive_steady_ns"] == 900000000
        with (first / "pico.csv").open(newline="") as stream:
            poses = list(csv.DictReader(stream))
        assert len(poses) == 3 and all(None not in p for p in poses), "CSV columns are misaligned"
        assert poses[0]["tracked"] == "1" and poses[0]["good_quality"] == "1"
        assert float(poses[0]["x_m"]) == 4.5 and float(poses[0]["qy"]) == 0.5
        assert poses[1]["available"] == "1" and poses[1]["tracked"] == "0" and float(poses[1]["x_m"]) == 8
        assert poses[2]["available"] == "0" and poses[2]["x_m"] == "" and poses[2]["params"] == ""
        summary = json.loads((first / "summary.json").read_text())
        assert summary["frames_written"] == 3 and summary["missing_source_frame_ids"] == 1
        assert summary["error"] == "" and summary["recording_drops"] == 0
        assert json.loads((second / "summary.json").read_text())["frames_written"] == 1
        overflow = next((root / "overflow").iterdir())
        summary = json.loads((overflow / "summary.json").read_text())
        assert summary["recording_drops"] == 1 and "overflow" in summary["error"]
        paced = next((root / "paced").iterdir())
        summary = json.loads((paced / "summary.json").read_text())
        assert summary["frames_written"] == 100 and summary["recording_drops"] == 0 and not summary["error"]
        frames = [json.loads(line) for line in (paced / "frames.jsonl").read_text().splitlines()]
        assert len(frames) == 100 and all(len(f["other_markers"]) == 1000 for f in frames)
        print("PASS: parsed all NatNet frame sections, Pico CSV, metadata, source gaps and incomplete-take summary.")


if __name__ == "__main__":
    check(sys.argv[1])
