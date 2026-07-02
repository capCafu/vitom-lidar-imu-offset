# motiv — one-command LiDAR↔IMU time-offset calibration (iKalibr wrapper)

A thin, self-contained wrapper around [iKalibr](https://github.com/Unsigned-Long/iKalibr)
that takes a **Robin recording folder** (a ROS2 `.mcap` + `sensors_metadata.yaml`) and
produces the **LiDAR↔IMU temporal offset**, then writes it back into the folder's
`sensors_metadata.yaml`.

It does **not** modify iKalibr; it drives the official solver in Docker.

## Requirements
- Docker only. Everything else (ROS Noetic, iKalibr, a Python 3.11 + `rosbags`
  environment for reading the `.mcap`, and `xvfb` for headless solving) is baked into
  the image.

## Usage
```bash
./calibrate_offset.sh /path/to/Robin_20260702_144602
```
First run builds the image (`ikalibr-motiv:latest`, a few minutes); later runs are fast.

Options (passed through to the solver):
| Option | Meaning |
|---|---|
| `--all` | use the whole bag (default: 60 s window starting at t = 5 s) |
| `--begin N --duration N` | custom time window (seconds from bag start) |
| `--no-patch` | print the result but don't modify `sensors_metadata.yaml` |
| `--lidar-type T` | force the iKalibr LiDAR type (else inferred from `lidar_model`) |
| `--gravity G` | gravity norm (default = measured mean \|acc\|) |
| `--keep-work` | keep the temp bag/config/output under `/tmp/motiv_work` |

## What it does
1. Reads `imu_topic`, `lidar_topics`, `lidar_model` from `sensors_metadata.yaml`.
2. Converts the `.mcap` → a ROS1 `.bag`:
   - IMU → `sensor_msgs/Imu` using the **gravity-included** acceleration (VectorNav
     `acceleration` field), gyro as-is, **no axis flip** (so the extrinsic is the true one).
   - LiDAR passed through unchanged (`PANDAR_XT_POINTS` reads it natively).
3. Generates a LiDAR+IMU-only iKalibr config and runs the solver headless.
4. Parses `TO_LkToBr` and reports the offset.

## Output & conventions
iKalibr reports `TO_LkToBr = τ`, defined by **`t_lidar + τ → IMU clock`** — this equals
your pipeline's `lidar_time_offset_sec`. The recording pipeline applies
`t_imu_used = t_imu_raw + imu_time_offset_sec`, so:

```
imu_time_offset_sec = −τ
```

The tool prints both, writes `lidar_imu_offset_result.yaml` into the folder, and (unless
`--no-patch`) sets `imu_time_offset_sec` in `sensors_metadata.yaml` (keeping a `.bak`).

## Notes
- **Excitation matters.** Record smooth, wide 3-axis rotations + translation; avoid violent
  motion. A 40–80 s well-excited window is plenty.
- **Precision floor.** If the IMU is software-timestamped (jittery inter-sample dt), the
  achievable offset precision is limited to a few ms regardless of tool — hardware
  timestamping the IMU is the real fix.
- Supported LiDAR mappings live in `LIDAR_TYPE_MAP` in `calibrate_offset.py`
  (Hesai XT → `PANDAR_XT_POINTS`, Ouster, Velodyne, Robosense). Extend as needed.
