import os
import glob
import datetime as dt
from dataclasses import dataclass
from typing import List, Optional, Dict, Tuple

import numpy as np


def read_calib_file(filepath: str) -> Dict[str, np.ndarray]:
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


def parse_kitti_timestamp(s: str) -> dt.datetime:
    s = s.strip()
    if "." in s:
        head, frac = s.split(".")
        frac = (frac + "000000")[:6]
        s = f"{head}.{frac}"
        return dt.datetime.strptime(s, "%Y-%m-%d %H:%M:%S.%f")
    return dt.datetime.strptime(s, "%Y-%m-%d %H:%M:%S")


def load_timestamps(filepath: str) -> List[dt.datetime]:
    with open(filepath, "r") as f:
        return [parse_kitti_timestamp(line) for line in f if line.strip()]


def datetime_to_seconds(ts: dt.datetime) -> float:
    return ts.timestamp()


def transform_from_rot_trans(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def rotx(t: float) -> np.ndarray:
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]], dtype=np.float64)


def roty(t: float) -> np.ndarray:
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], dtype=np.float64)


def rotz(t: float) -> np.ndarray:
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=np.float64)


EARTH_RADIUS = 6378137.0


def pose_from_oxts_packet(packet: List[float], scale: float) -> Tuple[np.ndarray, np.ndarray]:
    lat, lon, alt, roll, pitch, yaw = packet[:6]

    tx = scale * lon * np.pi * EARTH_RADIUS / 180.0
    ty = scale * EARTH_RADIUS * np.log(np.tan((90.0 + lat) * np.pi / 360.0))
    tz = alt
    t = np.array([tx, ty, tz], dtype=np.float64)

    Rx = rotx(roll)
    Ry = roty(pitch)
    Rz = rotz(yaw)
    R = Rz @ Ry @ Rx
    return R, t


def load_oxts_packets_and_poses(oxts_files: List[str]) -> List[np.ndarray]:
    packets = []
    for filename in oxts_files:
        with open(filename, "r") as f:
            vals = f.readline().strip().split()
            vals = [float(x) for x in vals]
            packets.append(vals)

    scale = None
    origin = None
    poses = []

    for packet in packets:
        lat = packet[0]
        if scale is None:
            scale = np.cos(lat * np.pi / 180.0)

        R, t = pose_from_oxts_packet(packet, scale)
        if origin is None:
            origin = t

        T_w_imu = transform_from_rot_trans(R, t - origin)
        poses.append(T_w_imu)

    return poses


def load_calib_rigid(filepath: str) -> np.ndarray:
    data = read_calib_file(filepath)

    if "R" in data and "T" in data:
        R = data["R"].reshape(3, 3)
        t = data["T"].reshape(3)
    elif "Tr" in data:
        Tr = data["Tr"].reshape(3, 4)
        R = Tr[:, :3]
        t = Tr[:, 3]
    else:
        raise RuntimeError(f"Unsupported calibration format in {filepath}")

    return transform_from_rot_trans(R, t)


def load_raw_calib(base_path: str, date: str) -> Dict[str, np.ndarray]:
    calib_dir = os.path.join(base_path, date)

    T_velo_imu = load_calib_rigid(os.path.join(calib_dir, "calib_imu_to_velo.txt"))
    T_imu_velo = np.linalg.inv(T_velo_imu)

    return {
        "T_velo_imu": T_velo_imu,
        "T_imu_velo": T_imu_velo,
    }


def compute_kitti_raw_point_times(points_xyz: np.ndarray,
                                  frame_time: dt.datetime,
                                  scan_duration: float = 0.1,
                                  front_azimuth: float = 0.0) -> np.ndarray:
    x = points_xyz[:, 0]
    y = points_xyz[:, 1]

    az = np.arctan2(y, x)
    az = np.mod(az, 2.0 * np.pi)

    rel = np.mod(az - front_azimuth, 2.0 * np.pi)

    frac = rel / (2.0 * np.pi)
    offsets = frac * scan_duration
    offsets = np.where(offsets > scan_duration * 0.5, offsets - scan_duration, offsets)

    return datetime_to_seconds(frame_time) + offsets


@dataclass
class KittiRawFrame:
    index: int
    filename: str
    frame_time: dt.datetime
    points_xyz: np.ndarray
    reflectance: np.ndarray
    point_timestamps: np.ndarray
    T_w_velo: np.ndarray


class KittiRawSequence:
    def __init__(self,
                 base_path: str,
                 date: str,
                 drive: str,
                 dataset: str = "sync",
                 frame_range: Optional[range] = None,
                 scan_duration: float = 0.1):
        self.base_path = base_path
        self.date = date
        self.drive = drive
        self.dataset = dataset
        self.scan_duration = scan_duration

        self.drive_name = f"{date}_drive_{drive}_{dataset}"
        self.data_path = os.path.join(base_path, date, self.drive_name)

        self.velodyne_path = os.path.join(self.data_path, "velodyne_points")
        self.oxts_path = os.path.join(self.data_path, "oxts")

        self.filenames = sorted(glob.glob(os.path.join(self.velodyne_path, "data", "*.bin")))
        self.oxts_files = sorted(glob.glob(os.path.join(self.oxts_path, "data", "*.txt")))

        self.velodyne_timestamps = load_timestamps(os.path.join(self.velodyne_path, "timestamps.txt"))
        self.oxts_timestamps = load_timestamps(os.path.join(self.oxts_path, "timestamps.txt"))

        n_velo = min(len(self.filenames), len(self.velodyne_timestamps))
        n_oxts = min(len(self.oxts_files), len(self.oxts_timestamps))

        if n_velo == 0:
            raise RuntimeError("No valid Velodyne files or timestamps found")
        if n_oxts == 0:
            raise RuntimeError("No valid OXTS files or timestamps found")

        self.filenames = self.filenames[:n_velo]
        self.velodyne_timestamps = self.velodyne_timestamps[:n_velo]

        self.oxts_files = self.oxts_files[:n_oxts]
        self.oxts_timestamps = self.oxts_timestamps[:n_oxts]

        self.calib = load_raw_calib(base_path, date)

        self.T_w_imu_all = load_oxts_packets_and_poses(self.oxts_files)
        self.gt_poses = [T_w_imu @ self.calib["T_imu_velo"] for T_w_imu in self.T_w_imu_all]

        n = min(len(self.filenames), len(self.gt_poses))
        self.filenames = self.filenames[:n]
        self.velodyne_timestamps = self.velodyne_timestamps[:n]
        self.gt_poses = self.gt_poses[:n]

        if frame_range is None:
            self.indices = list(range(n))
        else:
            self.indices = [i for i in frame_range if 0 <= i < n]

    def __len__(self) -> int:
        return len(self.indices)

    def _load_velodyne(self, filepath: str) -> Tuple[np.ndarray, np.ndarray]:
        scan = np.fromfile(filepath, dtype=np.float32).reshape((-1, 4))
        points_xyz = scan[:, :3].astype(np.float64)
        reflectance = scan[:, 3].astype(np.float64)
        return points_xyz, reflectance

    def get_frame(self, i: int) -> KittiRawFrame:
        idx = self.indices[i]

        points_xyz, reflectance = self._load_velodyne(self.filenames[idx])
        frame_time = self.velodyne_timestamps[idx]
        point_timestamps = compute_kitti_raw_point_times(
            points_xyz,
            frame_time=frame_time,
            scan_duration=self.scan_duration,
            front_azimuth=0.0,
        )

        return KittiRawFrame(
            index=idx,
            filename=self.filenames[idx],
            frame_time=frame_time,
            points_xyz=points_xyz,
            reflectance=reflectance,
            point_timestamps=point_timestamps,
            T_w_velo=self.gt_poses[idx],
        )

    def __iter__(self):
        for i in range(len(self)):
            yield self.get_frame(i)