import os
import glob
from dataclasses import dataclass
from typing import Optional

import numpy as np

def read_calib_file(filepath: str):
    data = {}
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            value = value.strip()
            try:
                data[key] = np.array([float(x) for x in value.split()], dtype=np.float64)
            except ValueError:
                pass
    return data


def load_kitti_odometry_calib(kitti_path: str, sequence: str):
    calib_path = os.path.join(kitti_path, "sequences", sequence, "calib.txt")
    filedata = read_calib_file(calib_path)

    calib = {}

    # Projection matrices
    if "P0" in filedata:
        calib["P_rect_00"] = filedata["P0"].reshape(3, 4)
    if "P1" in filedata:
        calib["P_rect_10"] = filedata["P1"].reshape(3, 4)
    if "P2" in filedata:
        calib["P_rect_20"] = filedata["P2"].reshape(3, 4)
    if "P3" in filedata:
        calib["P_rect_30"] = filedata["P3"].reshape(3, 4)

    if "Tr" not in filedata:
        raise RuntimeError(f"Missing Tr in calibration file: {calib_path}")

    T_cam0_velo = filedata["Tr"].reshape(3, 4)
    T_cam0_velo = np.vstack([T_cam0_velo, [0.0, 0.0, 0.0, 1.0]])
    calib["T_cam0_velo"] = T_cam0_velo

    return calib


def load_kitti_odometry_poses(kitti_path: str, sequence: str, T_cam0_velo: np.ndarray):
    pose_file = os.path.join(kitti_path, "poses", f"{sequence}.txt")
    if not os.path.exists(pose_file):
        raise RuntimeError(f"Pose file not found: {pose_file}")

    gt_poses = []
    with open(pose_file, "r") as f:
        for line in f:
            vals = np.fromstring(line.strip(), dtype=np.float64, sep=" ")
            if vals.size != 12:
                raise RuntimeError(f"Invalid pose line in {pose_file}: {line}")

            T_w_cam0 = vals.reshape(3, 4)
            T_w_cam0 = np.vstack([T_w_cam0, [0.0, 0.0, 0.0, 1.0]])

            # Convert pose from camera-0 frame to Velodyne frame
            T_w_velo = T_w_cam0 @ T_cam0_velo
            gt_poses.append(T_w_velo)

    return gt_poses


def load_kitti_bin_scan(filepath: str):
    scan = np.fromfile(filepath, dtype=np.float32)
    if scan.size % 4 != 0:
        raise RuntimeError(f"Invalid .bin scan shape in file: {filepath}")

    scan = scan.reshape((-1, 4))
    points_xyz = scan[:, :3].astype(np.float64)
    reflectance = scan[:, 3].astype(np.float64)
    return points_xyz, reflectance


@dataclass
class KittiOdometryFrame:
    index: int
    filename: str
    points_xyz: np.ndarray     # (N, 3)
    reflectance: np.ndarray    # (N,)
    T_w_velo: np.ndarray       # (4, 4)


class KittiOdometrySequence:
    def __init__(self, kitti_path: str, sequence: str, frame_range: Optional[range] = None):
        self.kitti_path = kitti_path
        self.sequence = sequence

        seq_path = os.path.join(kitti_path, "sequences", sequence)
        velo_path = os.path.join(seq_path, "velodyne")

        if not os.path.isdir(velo_path):
            raise RuntimeError(f"Velodyne folder not found: {velo_path}")

        self.filenames = sorted(glob.glob(os.path.join(velo_path, "*.bin")))
        if not self.filenames:
            raise RuntimeError(f"No .bin files found in: {velo_path}")

        self.calib = load_kitti_odometry_calib(kitti_path, sequence)
        self.gt_poses = load_kitti_odometry_poses(
            kitti_path,
            sequence,
            self.calib["T_cam0_velo"],
        )

        n = min(len(self.filenames), len(self.gt_poses))
        self.filenames = self.filenames[:n]
        self.gt_poses = self.gt_poses[:n]

        if frame_range is None:
            self.indices = list(range(n))
        else:
            self.indices = [i for i in frame_range if 0 <= i < n]

    def __len__(self):
        return len(self.indices)

    def get_frame(self, i: int) -> KittiOdometryFrame:
        idx = self.indices[i]
        filename = self.filenames[idx]
        points_xyz, reflectance = load_kitti_bin_scan(filename)

        return KittiOdometryFrame(
            index=idx,
            filename=filename,
            points_xyz=points_xyz,
            reflectance=reflectance,
            T_w_velo=self.gt_poses[idx],
        )

    def __iter__(self):
        for i in range(len(self)):
            yield self.get_frame(i)