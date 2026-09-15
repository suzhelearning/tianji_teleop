"""Observation-only collection opt-in; no SDKs or hardware are used."""
import pytest
from real_robot.run_teleop import main


def test_dataset_recording_cannot_run_in_dry_or_partial_device_mode(monkeypatch, tmp_path):
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    for args in (["--dataset", str(tmp_path), "--task", "test"],
                 ["--confirm-real", "--devices", "arms", "--dataset", str(tmp_path), "--task", "test"]):
        with pytest.raises(SystemExit) as error:
            main(args)
        assert error.value.code == 2
