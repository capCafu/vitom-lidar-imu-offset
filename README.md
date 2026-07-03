# vitom-lidar-imu-offset

**One command → the LiDAR↔IMU time offset for a recording.**

Point it at a recording folder (a ROS 2 `rosbag2` `.mcap` + `sensors_metadata.yaml`)
and it returns the temporal offset between the LiDAR and the IMU, then writes it back
into the folder's `sensors_metadata.yaml`.

It's a thin, self-contained wrapper around
[iKalibr](https://github.com/Unsigned-Long/iKalibr) (targetless, continuous-time
spatiotemporal calibration). It does **not** modify iKalibr — it drives the official
solver in Docker and handles all the plumbing (mcap → ROS 1 bag conversion, config
generation, headless solving, result parsing, unit/convention conversion).

---

## Requirements

- **Docker only.** Everything else — ROS Noetic, iKalibr, a Python 3.11 + `rosbags`
  environment to read the `.mcap`, and `xvfb` for headless solving — is baked into the
  image. First run builds the image (a few minutes); later runs are fast.

## Quick start

```bash
git clone https://github.com/capCafu/vitom-lidar-imu-offset
cd vitom-lidar-imu-offset
./calibrate_offset.sh /path/to/Robin_2026****_******
```

Example output:

```
================================================================
 LiDAR <-> IMU TIME OFFSET  (iKalibr, motiv wrapper)
================================================================
  lidar_time_offset_sec (tau)   =  +0.006622 s   (t_lidar + tau -> IMU clock)
  imu_time_offset_sec  (=-tau)  =  -0.006622 s   <- value for sensors_metadata.yaml
  extrinsic rotation LiDAR->IMU =  179.965 deg (raw frames)
  extrinsic translation (m)     =  [-0.0152, +0.0012, -0.0006]
  gravity norm (quality check)  =  9.8059 m/s^2 (expect ~9.8)
================================================================
```

The tool writes `lidar_imu_offset_result.yaml` into the folder and (unless `--no-patch`)
sets `imu_time_offset_sec` in `sensors_metadata.yaml`, keeping a `.bak`.

## Options

| Option | Meaning |
|---|---|
| `--all` | use the whole bag (default: 60 s window from t = 5 s) |
| `--begin N --duration N` | custom time window (seconds from bag start) |
| `--no-patch` | print + write result file, but don't touch `sensors_metadata.yaml` |
| `--lidar-type T` | force the iKalibr LiDAR type (else inferred from `lidar_model`) |
| `--gravity G` | gravity norm (default = measured mean \|acc\|) |
| `--keep-work` | keep the temp bag/config/output under `/tmp/motiv_work` |

## How it works

1. Reads `imu_topic`, `lidar_topics`, `lidar_model` from `sensors_metadata.yaml`.
2. Converts the `.mcap` → a ROS 1 `.bag`:
   - IMU → `sensor_msgs/Imu` using the **gravity-included** acceleration
     (VectorNav `acceleration` field), gyro as-is, **no axis flip** (so the extrinsic is
     the true physical one).
   - LiDAR passed through unchanged.
3. Generates a LiDAR + IMU only iKalibr config and runs the solver headless.
4. Parses `TO_LkToBr`, converts conventions, prints, and patches the YAML.

## Output & conventions

iKalibr reports `TO_LkToBr = τ`, defined by **`t_lidar + τ → IMU clock`** — the same as a
`lidar_time_offset_sec`. Since the recording pipeline applies
`t_imu_used = t_imu_raw + imu_time_offset_sec`:

```
imu_time_offset_sec = −τ
```

Both are printed; `imu_time_offset_sec` is the value written into `sensors_metadata.yaml`.

## Notes

- **Excitation matters.** Record smooth, wide 3-axis rotations + some translation; avoid
  violent motion. A 40–80 s well-excited window is plenty. (Too-violent motion can break
  the solve; too-short recordings won't converge.)
- **Precision floor.** If the IMU is *software*-timestamped (jittery inter-sample dt) while
  the LiDAR is hardware/PTP-stamped, the achievable offset precision is limited to a few ms
  regardless of tool — hardware-timestamping the IMU is the real fix.
- **Supported LiDARs** live in `LIDAR_TYPE_MAP` in `calibrate_offset.py`
  (Hesai XT → `PANDAR_XT_POINTS`, Ouster, Velodyne, Robosense). Extend as needed, or pass
  `--lidar-type`.

## Credits & license

Built on **iKalibr** by Shuolong Chen et al. (Wuhan University) —
<https://github.com/Unsigned-Long/iKalibr> (see their paper, *IEEE T-RO 2025*). This
wrapper adds only the folder-in / offset-out automation; all calibration is done by
iKalibr. Please cite iKalibr if you use this in academic work.
