"""Loaders: run folder parsing + manifest, summary, CLI invocation."""

from __future__ import annotations

import json

import pandas as pd
import pytest
from gncpost.loaders import load_run, load_summary, run_cli


def _write_fake_run(tmp_path):
    """Build a minimal valid run folder."""
    for name, cols in (
        ("vehicle.csv", "t,x,y,z,vx,vy,vz,roll,pitch,yaw,mass,mach"),
        ("target.csv", "t,x,y,z,vx,vy,vz"),
        (
            "gnc.csv",
            "t,accel_cmd_x,accel_cmd_y,accel_cmd_z,fin_x,fin_y,fin_z,"
            "los_angle,los_rate,v_closing,range,nav_x,nav_y,nav_z",
        ),
        (
            "sensors.csv",
            "t,imu_accel_true_x,imu_accel_meas_x,imu_gyro_true_x,"
            "imu_gyro_meas_x,seeker_los_true,seeker_los_meas",
        ),
    ):
        ncols = len(cols.split(","))
        rows = [cols] + [",".join(["0"] * ncols), ",".join([str(i) for i in range(ncols)])]
        (tmp_path / name).write_text("\n".join(rows) + "\n")
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "scenario": "t",
                "model": "3dof",
                "seed": 1,
                "intercept": True,
                "miss_distance": 1.23,
                "intercept_time": 4.5,
                "num_frames": 2,
            }
        )
    )


def test_load_run(tmp_path):
    _write_fake_run(tmp_path)
    run = load_run(tmp_path)
    assert list(run.vehicle.columns)[:4] == ["t", "x", "y", "z"]
    assert run.intercept is True
    assert run.miss_distance == pytest.approx(1.23)
    assert len(run.t) == 2
    assert "imu_accel_meas_x" in run.sensors.columns


def test_load_run_missing_folder(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_run(tmp_path / "does_not_exist")


def test_load_summary(tmp_path):
    df = pd.DataFrame(
        {
            "case": [0, 1],
            "seed": [1, 2],
            "miss_distance": [0.5, 1.5],
            "intercept_time": [4.0, 4.2],
            "intercept": [1, 1],
        }
    )
    df.to_csv(tmp_path / "summary.csv", index=False)
    loaded = load_summary(tmp_path)
    assert len(loaded) == 2
    assert loaded["miss_distance"].tolist() == [0.5, 1.5]


def test_run_cli_missing_binary(tmp_path):
    with pytest.raises(FileNotFoundError):
        run_cli(tmp_path / "cfg.json", tmp_path / "out", cli_path=tmp_path / "nope")
