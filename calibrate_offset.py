#!/usr/bin/env python3
"""
calibrate_offset.py  (motiv / iKalibr wrapper)
==============================================
One-command LiDAR<->IMU *time offset* calibration for a Robin recording folder.

Given a folder containing a ROS2 rosbag2 (.mcap) plus `sensors_metadata.yaml`, this:
  1. reads the sensor topics/model from sensors_metadata.yaml,
  2. converts the .mcap -> a ROS1 .bag (IMU re-wrapped to sensor_msgs/Imu using the
     gravity-included acceleration; LiDAR passed through unchanged; NO axis flip),
  3. runs iKalibr (LiDAR + IMU only) headless to estimate the temporal offset,
  4. prints the offset in both conventions and (unless --no-patch) writes
     `imu_time_offset_sec` into the folder's sensors_metadata.yaml (keeping a .bak),
  5. writes a small `lidar_imu_offset_result.yaml` into the folder.

Conventions
-----------
iKalibr reports TO_LkToBr = tau, defined by:  t_lidar + tau  ->  IMU clock.
That equals your pipeline's `lidar_time_offset_sec`. Since the extractor applies
`t_imu_used = t_imu_raw + imu_time_offset_sec`, we have:
        imu_time_offset_sec = -tau

Run inside the image:  docker run --rm -v /path/to/Robin_folder:/folder ikalibr-motiv /folder
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
from math import sqrt
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

TEMPLATE = "/opt/motiv/ikalibr-config.template.yaml"
RUN_SH = "/opt/motiv/run_ikalibr.sh"
IMU_OUT_TOPIC = "/imu/data"

# map (lidar_type, lidar_model) or model substring -> iKalibr LiDAR type string
LIDAR_TYPE_MAP = {
    "xt32": "PANDAR_XT_POINTS",
    "xt16": "PANDAR_XT_POINTS",
    "pandar": "PANDAR_XT_POINTS",
    "ouster": "OUSTER_POINTS",
    "vlp": "VLP_POINTS",
    "velodyne": "VLP_POINTS",
    "rslidar": "RSLIDAR_POINTS",
    "robosense": "RSLIDAR_POINTS",
}


def log(msg):
    print(f"[motiv] {msg}", flush=True)


def die(msg, code=2):
    print(f"[motiv][ERROR] {msg}", file=sys.stderr, flush=True)
    sys.exit(code)


def resolve_lidar_type(lidar_type, lidar_model, override):
    if override:
        return override
    hay = f"{lidar_model} {lidar_type}".lower()
    for key, val in LIDAR_TYPE_MAP.items():
        if key in hay:
            return val
    return None


def read_metadata(folder):
    sm = folder / "sensors_metadata.yaml"
    if not sm.exists():
        die(f"sensors_metadata.yaml not found in {folder}")
    meta = yaml.safe_load(sm.read_text())
    imu_topic = meta.get("imu_topic") or "/vectornav/IMU"
    lidar_topics = meta.get("lidar_topics") or ["/lidar_points"]
    lidar_topic = lidar_topics[0]
    lidar_type = str(meta.get("lidar_type", "")).lower()
    lidar_model = str(meta.get("lidar_model", "")).lower()
    return sm, imu_topic, lidar_topic, lidar_type, lidar_model


def convert(folder, imu_in, lidar_in, out_bag):
    """ROS2 .mcap -> ROS1 .bag. IMU -> sensor_msgs/Imu (accel WITH gravity), no flip."""
    ts = get_typestore(Stores.ROS1_NOETIC)
    Imu = ts.types["sensor_msgs/msg/Imu"]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    Vector3 = ts.types["geometry_msgs/msg/Vector3"]
    Quaternion = ts.types["geometry_msgs/msg/Quaternion"]
    PointCloud2 = ts.types["sensor_msgs/msg/PointCloud2"]
    PointField = ts.types["sensor_msgs/msg/PointField"]
    IMU_TYPE = "sensor_msgs/msg/Imu"
    PC2_TYPE = "sensor_msgs/msg/PointCloud2"
    imu_def, imu_md5 = ts.generate_msgdef(IMU_TYPE)
    pc2_def, pc2_md5 = ts.generate_msgdef(PC2_TYPE)
    zeros9 = np.zeros(9, dtype=np.float64)
    no_orient = np.array([-1.0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)

    n_imu = n_lidar = 0
    acc_sum = 0.0
    t_first = t_last = None

    with AnyReader([folder]) as reader, Writer(out_bag) as writer:
        conns = {c.topic: c for c in reader.connections}
        if imu_in not in conns:
            die(f"IMU topic '{imu_in}' not in bag. Topics: {list(conns)}")
        if lidar_in not in conns:
            die(f"LiDAR topic '{lidar_in}' not in bag. Topics: {list(conns)}")
        imu_msgtype = conns[imu_in].msgtype
        use_accel_field = "vectornav" in imu_msgtype.lower()
        log(f"IMU input type: {imu_msgtype} -> "
            f"{'acceleration (with gravity)' if use_accel_field else 'linear_acceleration'} field")

        imu_conn = writer.add_connection(IMU_OUT_TOPIC, IMU_TYPE, msgdef=imu_def, md5sum=imu_md5)
        lidar_conn = writer.add_connection(lidar_in, PC2_TYPE, msgdef=pc2_def, md5sum=pc2_md5)

        for conn, t, raw in reader.messages():
            if conn.topic == imu_in:
                m = reader.deserialize(raw, conn.msgtype)
                gyr = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
                if use_accel_field:
                    a = m.acceleration
                else:
                    a = m.linear_acceleration
                acc = np.array([a.x, a.y, a.z])
                acc_sum += sqrt(float(acc @ acc))
                out = Imu(
                    header=Header(seq=0,
                                  stamp=Time(sec=int(m.header.stamp.sec),
                                             nanosec=int(m.header.stamp.nanosec)),
                                  frame_id=m.header.frame_id or "imu"),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=no_orient.copy(),
                    angular_velocity=Vector3(x=gyr[0], y=gyr[1], z=gyr[2]),
                    angular_velocity_covariance=zeros9.copy(),
                    linear_acceleration=Vector3(x=acc[0], y=acc[1], z=acc[2]),
                    linear_acceleration_covariance=zeros9.copy(),
                )
                writer.write(imu_conn, t, ts.serialize_ros1(out, IMU_TYPE))
                n_imu += 1
                t_first = t if t_first is None else t_first
                t_last = t
            elif conn.topic == lidar_in:
                m = reader.deserialize(raw, conn.msgtype)
                out_pc = PointCloud2(
                    header=Header(seq=0,
                                  stamp=Time(sec=int(m.header.stamp.sec),
                                             nanosec=int(m.header.stamp.nanosec)),
                                  frame_id=m.header.frame_id or "lidar"),
                    height=int(m.height), width=int(m.width),
                    fields=[PointField(name=f.name, offset=int(f.offset),
                                       datatype=int(f.datatype), count=int(f.count))
                            for f in m.fields],
                    is_bigendian=bool(m.is_bigendian), point_step=int(m.point_step),
                    row_step=int(m.row_step), data=m.data, is_dense=bool(m.is_dense),
                )
                writer.write(lidar_conn, t, ts.serialize_ros1(out_pc, PC2_TYPE))
                n_lidar += 1

    mean_acc = acc_sum / max(n_imu, 1)
    dur = (t_last - t_first) / 1e9 if (t_first and t_last) else 0.0
    log(f"converted: {n_imu} IMU, {n_lidar} LiDAR frames | mean|acc|={mean_acc:.3f} m/s^2 | bag ~{dur:.1f}s")
    if not (8.5 < mean_acc < 11.0):
        log(f"WARNING: mean|acc|={mean_acc:.3f} is far from 9.8 - IMU accel may be gravity-compensated "
            f"or in wrong units; the solve may be poor.")
    return mean_acc, dur


def write_config(template, out_path, imu_topic, lidar_topic, lidar_type, bag, out_dir, begin, duration, gravity):
    txt = Path(template).read_text()
    repl = {
        "__IMU_TOPIC__": imu_topic,
        "__LIDAR_TOPIC__": lidar_topic,
        "__LIDAR_TYPE__": lidar_type,
        "__BAG__": bag,
        "__OUTPUT__": out_dir,
        "__BEGIN__": str(begin),
        "__DURATION__": str(duration),
        "__GRAVITY__": f"{gravity:.4f}",
    }
    for k, v in repl.items():
        txt = txt.replace(k, v)
    Path(out_path).write_text(txt)


def quat_to_axis_angle_deg(qx, qy, qz, qw):
    n = sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    ang = 2.0 * np.degrees(np.arccos(min(1.0, abs(qw))))
    return ang


def parse_result(param_yaml, lidar_topic):
    p = yaml.safe_load(Path(param_yaml).read_text())["CalibParam"]

    def find(section, key):
        for kv in (section or []):
            if kv.get("key") == key:
                return kv.get("value")
        return None

    tau = find(p["TEMPORAL"]["TO_LkToBr"], lidar_topic)
    if tau is None:
        die(f"TO_LkToBr for '{lidar_topic}' not found in {param_yaml}")
    so3 = find(p["EXTRI"]["SO3_LkToBr"], lidar_topic) or {}
    pos = find(p["EXTRI"]["POS_LkInBr"], lidar_topic) or {}
    g = p.get("GRAVITY", {})
    gnorm = sqrt(sum(float(g.get(f"r{i}c0", 0.0)) ** 2 for i in range(3)))
    rot_deg = quat_to_axis_angle_deg(so3.get("qx", 0), so3.get("qy", 0), so3.get("qz", 0), so3.get("qw", 1))
    trans = [float(pos.get(f"r{i}c0", 0.0)) for i in range(3)]
    return float(tau), rot_deg, trans, gnorm


def patch_metadata(sm_path, imu_off):
    text = sm_path.read_text()
    shutil.copyfile(sm_path, sm_path.with_suffix(sm_path.suffix + ".bak"))
    line = f"imu_time_offset_sec: {imu_off:.6f}"
    if re.search(r"^imu_time_offset_sec:.*$", text, flags=re.M):
        text = re.sub(r"^imu_time_offset_sec:.*$", line, text, flags=re.M)
    else:
        text = text.rstrip("\n") + "\n" + line + "\n"
    sm_path.write_text(text)


def chown_like(target, ref):
    try:
        st = os.stat(ref)
        os.chown(target, st.st_uid, st.st_gid)
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="LiDAR<->IMU time offset via iKalibr (motiv wrapper).")
    ap.add_argument("folder", help="Robin recording folder (.mcap + sensors_metadata.yaml)")
    ap.add_argument("--begin", type=float, default=5.0, help="window start (s from bag start); default 5")
    ap.add_argument("--duration", type=float, default=60.0, help="window length (s); default 60")
    ap.add_argument("--all", action="store_true", help="use the whole bag (overrides --begin/--duration)")
    ap.add_argument("--gravity", type=float, default=None, help="gravity norm (m/s^2); default = measured |acc|")
    ap.add_argument("--lidar-type", default=None, help="override iKalibr LiDAR type (e.g. PANDAR_XT_POINTS)")
    ap.add_argument("--no-patch", action="store_true", help="do not modify sensors_metadata.yaml")
    ap.add_argument("--keep-work", action="store_true", help="keep the temp work dir (bag/config/output)")
    args = ap.parse_args()

    folder = Path(args.folder).resolve()
    if not folder.is_dir():
        die(f"not a folder: {folder}")

    sm, imu_topic, lidar_topic, lidar_type, lidar_model = read_metadata(folder)
    ltype = resolve_lidar_type(lidar_type, lidar_model, args.lidar_type)
    if not ltype:
        die(f"cannot map LiDAR (type='{lidar_type}', model='{lidar_model}') to an iKalibr type. "
            f"Pass --lidar-type explicitly.")
    log(f"folder={folder}")
    log(f"imu_topic={imu_topic}  lidar_topic={lidar_topic}  lidar_type={ltype}")

    work = Path("/tmp/motiv_work")
    if work.exists():
        shutil.rmtree(work)
    (work / "output").mkdir(parents=True)
    bag = str(work / "input.bag")
    cfg = str(work / "config.yaml")
    out_dir = str(work / "output")

    log("converting mcap -> ROS1 bag ...")
    mean_acc, dur = convert(folder, imu_topic, lidar_topic, bag)
    gravity = args.gravity if args.gravity is not None else (mean_acc if 8.5 < mean_acc < 11.0 else 9.8)

    if args.all:
        begin, duration = -1, -1
    else:
        begin = args.begin
        duration = args.duration
        if dur and begin + duration > dur:
            duration = max(10.0, dur - begin - 1.0)
            log(f"clamping window to available data: begin={begin}s duration={duration:.1f}s")

    write_config(TEMPLATE, cfg, imu_topic if False else IMU_OUT_TOPIC, lidar_topic, ltype,
                 bag, out_dir, begin, duration, gravity)
    log(f"running iKalibr (window: begin={begin}, duration={duration}, gravity={gravity:.3f}) ...")

    r = subprocess.run(["bash", RUN_SH, cfg, str(work / "ikalibr_solve.log")])
    param = Path(out_dir) / "ikalibr_param.yaml"
    if r.returncode != 0 or not param.exists():
        tail = ""
        lg = work / "ikalibr_solve.log"
        if lg.exists():
            tail = "\n".join(lg.read_text().splitlines()[-40:])
        die(f"iKalibr did not finish successfully.\n--- last solver log lines ---\n{tail}")

    tau, rot_deg, trans, gnorm = parse_result(param, lidar_topic)
    imu_off = -tau

    print("\n" + "=" * 64)
    print(" LiDAR <-> IMU TIME OFFSET  (iKalibr, motiv wrapper)")
    print("=" * 64)
    print(f"  lidar_time_offset_sec (tau)   =  {tau:+.6f} s   (t_lidar + tau -> IMU clock)")
    print(f"  imu_time_offset_sec  (=-tau)  =  {imu_off:+.6f} s   <- value for sensors_metadata.yaml")
    print(f"  extrinsic rotation LiDAR->IMU =  {rot_deg:.3f} deg (raw frames)")
    print(f"  extrinsic translation (m)     =  [{trans[0]:+.4f}, {trans[1]:+.4f}, {trans[2]:+.4f}]")
    print(f"  gravity norm (quality check)  =  {gnorm:.4f} m/s^2 (expect ~9.8)")
    print("=" * 64 + "\n")

    # write a result record into the folder
    res = folder / "lidar_imu_offset_result.yaml"
    res.write_text(
        "# LiDAR<->IMU time offset (iKalibr, motiv wrapper)\n"
        f"lidar_time_offset_sec: {tau:.6f}   # t_lidar + tau -> IMU clock\n"
        f"imu_time_offset_sec: {imu_off:.6f}  # = -tau ; goes into sensors_metadata.yaml\n"
        f"extrinsic_rotation_deg: {rot_deg:.3f}\n"
        f"extrinsic_translation_m: [{trans[0]:.4f}, {trans[1]:.4f}, {trans[2]:.4f}]\n"
        f"gravity_norm: {gnorm:.4f}\n"
    )
    chown_like(res, folder)

    if args.no_patch:
        log("--no-patch: sensors_metadata.yaml left unchanged.")
    else:
        patch_metadata(sm, imu_off)
        chown_like(sm, folder)
        chown_like(sm.with_suffix(sm.suffix + ".bak"), folder)
        log(f"patched {sm.name}: imu_time_offset_sec = {imu_off:.6f} (backup: {sm.name}.bak)")

    if not args.keep_work:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    main()
