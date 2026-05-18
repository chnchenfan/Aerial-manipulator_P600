#!/usr/bin/env python3
import argparse
import os
from datetime import datetime

import numpy as np
import rosbag
from scipy.io import savemat


TOPIC_POSE = "/mavros/local_position/pose"
TOPIC_POSE_D = "/wjl/guidefly/pose_d"
TOPIC_ARM_D = "/wjl/arm/guidefly/angle_d"
TOPIC_ARM_R = "/wjl/arm/real/angle_r"


def interp_series(rows, t_query, width):
    if not rows:
        return np.zeros((len(t_query), width))
    arr = np.array(rows, dtype=float)
    out = np.zeros((len(t_query), width))
    for i in range(width):
        out[:, i] = np.interp(t_query, arr[:, 0], arr[:, i + 1])
    return out


def signal(time, data):
    return {"time": np.asarray(time, dtype=float).reshape(-1), "data": np.asarray(data, dtype=float)}


def compute_metrics(p_true, p_desired, q_true, q_desired):
    p_error = p_true - p_desired
    p_norm = np.linalg.norm(p_error, axis=1)
    q_error = q_true - q_desired
    q_norm = np.linalg.norm(q_error, axis=1)
    return {
        "position_mean": float(np.mean(p_norm)),
        "position_rms": float(np.sqrt(np.mean(p_norm ** 2))),
        "position_max": float(np.max(p_norm)),
        "position_axis_mean": np.mean(np.abs(p_error), axis=0),
        "position_axis_rms": np.sqrt(np.mean(p_error ** 2, axis=0)),
        "position_axis_max": np.max(np.abs(p_error), axis=0),
        "arm_axis_max": np.max(np.abs(q_error), axis=0),
        "arm_axis_rms": np.sqrt(np.mean(q_error ** 2, axis=0)),
        "arm_rms": float(np.sqrt(np.mean(q_norm ** 2))),
        "arm_max": float(np.max(q_norm)),
        "is_divergent": False,
        "thrust_saturation_ratio": 0.0,
        "torque_saturation_ratio": 0.0,
    }


def export_window(bag_path, start_s, end_s, experiment, mode, output_file):
    pose_rows = []
    pose_d_rows = []
    arm_d_rows = []
    arm_r_rows = []

    with rosbag.Bag(bag_path) as bag:
        t0 = bag.get_start_time()
        topics = [TOPIC_POSE, TOPIC_POSE_D, TOPIC_ARM_D, TOPIC_ARM_R]
        for topic, msg, t in bag.read_messages(topics=topics):
            tt = t.to_sec() - t0
            if topic == TOPIC_POSE:
                p = msg.pose.position
                pose_rows.append((tt, p.x, p.y, p.z))
            elif topic == TOPIC_POSE_D and start_s <= tt <= end_s:
                pose_d_rows.append((tt, msg.x_d, msg.y_d, msg.z_d))
            elif topic == TOPIC_ARM_D:
                arm_d_rows.append((tt, msg.arm1_angle, msg.arm2_angle, msg.hand_angle))
            elif topic == TOPIC_ARM_R:
                arm_r_rows.append((tt, msg.arm1_angle, msg.arm2_angle, msg.hand_angle))

    if not pose_d_rows:
        raise RuntimeError(f"No {TOPIC_POSE_D} samples in window {start_s}-{end_s}")

    pose_d = np.array(pose_d_rows, dtype=float)
    sample_t_abs = pose_d[:, 0]
    sample_t = sample_t_abs - sample_t_abs[0]
    p_desired = pose_d[:, 1:4]
    p_true = interp_series(pose_rows, sample_t_abs, 3)
    q_desired = interp_series(arm_d_rows, sample_t_abs, 3)
    q_true = interp_series(arm_r_rows, sample_t_abs, 3)
    f = np.zeros((len(sample_t), 1))
    tau = np.zeros((len(sample_t), 3))

    metrics = compute_metrics(p_true, p_desired, q_true, q_desired)
    result = {
        "config": {
            "source_bag": bag_path,
            "experiment": experiment,
            "mode": mode,
            "window_start_s": float(start_s),
            "window_end_s": float(end_s),
            "sample_count": int(len(sample_t)),
        },
        "signals": {
            "p_true": signal(sample_t, p_true),
            "p_actual": signal(sample_t, p_true),
            "p_desired": signal(sample_t, p_desired),
            "q_true": signal(sample_t, q_true),
            "q_desired": signal(sample_t, q_desired),
            "f": signal(sample_t, f.reshape(-1)),
            "tau": signal(sample_t, tau),
        },
        "metrics": metrics,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "output_file": output_file,
    }

    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    savemat(output_file, {"result": result}, do_compression=True)
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--exp1-bag", default="/media/cf/Data1/课程/课内课程/毕业设计/data_P600/raw/source_20260514/exp1_px4.bag")
    parser.add_argument("--exp3-bag", default="/media/cf/Data1/课程/课内课程/毕业设计/data_P600/raw/source_20260515_215540/exp3_px4.bag")
    parser.add_argument("--output-dir", default="/media/cf/Data1/课程/课内课程/毕业设计/data_P600/raw")
    args = parser.parse_args()

    jobs = [
        (args.exp1_bag, "exp1", "mode1", 61.11, 91.61, "p600_mode1_exp1_paper_eso.mat"),
        (args.exp3_bag, "exp3", "mode2", 47.80, 138.33, "p600_mode2_exp3_paper_eso.mat"),
    ]
    for bag_path, experiment, mode, start_s, end_s, filename in jobs:
        out = os.path.join(args.output_dir, filename)
        metrics = export_window(bag_path, start_s, end_s, experiment, mode, out)
        print(
            f"{experiment} {out}: "
            f"position_mean={metrics['position_mean']:.6f} "
            f"position_rms={metrics['position_rms']:.6f} "
            f"position_max={metrics['position_max']:.6f}"
        )


if __name__ == "__main__":
    main()
