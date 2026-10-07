from dataclasses import dataclass
from collections import deque
from typing import Dict, Sequence

import time
import numpy as np
import pygicp
from matplotlib import pyplot


@dataclass
class SegmentMetric:
    count: int
    t_err_percent_mean: float
    r_err_deg_per_m_mean: float


@dataclass
class OverallMetric:
    t_err_percent_mean: float
    r_err_deg_per_m_mean: float
    num_samples: int


@dataclass
class KittiEvaluationResult:
    per_segment: Dict[int, SegmentMetric]
    overall: OverallMetric

    def print(self) -> None:
        print("\nKITTI drift metrics:")
        print("Length[m] | samples | t_err[%] | r_err[deg/m]")
        for length in sorted(self.per_segment.keys()):
            metric = self.per_segment[length]
            print(
                f"{length:8d} | {metric.count:7d} | "
                f"{metric.t_err_percent_mean:7.3f} | "
                f"{metric.r_err_deg_per_m_mean:10.6f}"
            )

        print(
            f"\nOverall: "
            f"t_err[%]={self.overall.t_err_percent_mean:.3f}, "
            f"r_err[deg/m]={self.overall.r_err_deg_per_m_mean:.6f}, "
            f"samples={self.overall.num_samples}"
        )


class KittiOdometryEvaluator:
    def __init__(self, segment_lengths: Sequence[int] = (100, 200, 300, 400, 500, 600, 700, 800)):
        self.segment_lengths = tuple(segment_lengths)

    @staticmethod
    def pose_distance(p1: np.ndarray, p2: np.ndarray) -> float:
        return float(np.linalg.norm(p1[:3, 3] - p2[:3, 3]))

    @classmethod
    def trajectory_distances(cls, poses: Sequence[np.ndarray]) -> np.ndarray:
        dists = [0.0]
        for i in range(1, len(poses)):
            dists.append(dists[-1] + cls.pose_distance(poses[i - 1], poses[i]))
        return np.asarray(dists, dtype=np.float64)

    @staticmethod
    def rotation_error_rad(T_err: np.ndarray) -> float:
        R = T_err[:3, :3]
        c = (np.trace(R) - 1.0) * 0.5
        c = np.clip(c, -1.0, 1.0)
        return float(np.arccos(c))

    def evaluate(
        self,
        est_poses: Sequence[np.ndarray],
        gt_poses: Sequence[np.ndarray],
    ) -> KittiEvaluationResult:
        assert len(est_poses) == len(gt_poses), "est_poses and gt_poses must have same length"

        n = len(gt_poses)
        gt_dists = self.trajectory_distances(gt_poses)

        results = {
            length: {"t_err_percent": [], "r_err_deg_per_m": []}
            for length in self.segment_lengths
        }

        for i in range(n):
            for length in self.segment_lengths:
                target_dist = gt_dists[i] + length
                j = np.searchsorted(gt_dists, target_dist, side="left")
                if j >= n:
                    continue

                T_gt_rel = np.linalg.inv(gt_poses[i]) @ gt_poses[j]
                T_est_rel = np.linalg.inv(est_poses[i]) @ est_poses[j]
                T_err = np.linalg.inv(T_gt_rel) @ T_est_rel

                t_err = np.linalg.norm(T_err[:3, 3])
                r_err = self.rotation_error_rad(T_err)

                results[length]["t_err_percent"].append((t_err / length) * 100.0)
                results[length]["r_err_deg_per_m"].append((r_err / length) * (180.0 / np.pi))

        per_segment = {}
        all_t = []
        all_r = []

        for length in self.segment_lengths:
            t_arr = np.asarray(results[length]["t_err_percent"], dtype=np.float64)
            r_arr = np.asarray(results[length]["r_err_deg_per_m"], dtype=np.float64)

            if t_arr.size:
                all_t.append(t_arr)
            if r_arr.size:
                all_r.append(r_arr)

            per_segment[length] = SegmentMetric(
                count=int(t_arr.size),
                t_err_percent_mean=float(t_arr.mean()) if t_arr.size else float("nan"),
                r_err_deg_per_m_mean=float(r_arr.mean()) if r_arr.size else float("nan"),
            )

        all_t = np.concatenate(all_t) if all_t else np.array([], dtype=np.float64)
        all_r = np.concatenate(all_r) if all_r else np.array([], dtype=np.float64)

        overall = OverallMetric(
            t_err_percent_mean=float(all_t.mean()) if all_t.size else float("nan"),
            r_err_deg_per_m_mean=float(all_r.mean()) if all_r.size else float("nan"),
            num_samples=int(all_t.size),
        )

        return KittiEvaluationResult(per_segment=per_segment, overall=overall)


class FpsTracker:
    def __init__(self, window_size: int = 10):
        self.window_size = window_size
        self.timestamps = deque()
        self.values = []

    def tick(self) -> float:
        now = time.time()
        self.timestamps.append(now)

        while len(self.timestamps) > self.window_size:
            self.timestamps.popleft()

        if len(self.timestamps) >= 2:
            fps = (len(self.timestamps) - 1) / (self.timestamps[-1] - self.timestamps[0])
            self.values.append(fps)
            return fps

        return 0.0

    @property
    def average(self) -> float:
        if not self.values:
            return 0.0
        return float(sum(self.values) / len(self.values))


@dataclass
class OdometryRunResult:
    est_poses: Sequence[np.ndarray]
    gt_poses: Sequence[np.ndarray]
    avg_fps: float
    evaluation: KittiEvaluationResult


def to_local_frame(poses: Sequence[np.ndarray]) -> Sequence[np.ndarray]:
    assert len(poses) > 0, "poses must not be empty"
    T0_inv = np.linalg.inv(poses[0])
    return [T0_inv @ T for T in poses]


def run_lo(
    reg,
    dataset,
    downsample_resolution: float = 0.5,
    segment_lengths: Sequence[int] = (100, 200, 300, 400, 500, 600, 700, 800),
) -> OdometryRunResult:
    fps_tracker = FpsTracker(window_size=10)
    evaluator = KittiOdometryEvaluator(segment_lengths=segment_lengths)

    est_poses = []
    last_delta = np.eye(4, dtype=np.float64)

    gt_poses_global = dataset.gt_poses
    gt_poses_local = to_local_frame(gt_poses_global)

    for sample in dataset:
        points = sample.points_xyz
        if downsample_resolution > 0.0:
            points = pygicp.downsample(points, downsample_resolution)

        if sample.index == 0:
            reg.set_input_target(points)
            est_poses.append(np.eye(4, dtype=np.float64))
            continue

        reg.set_input_source(points)
        delta = reg.align(initial_guess=last_delta)
        reg.swap_source_and_target()

        est_poses.append(est_poses[-1] @ delta)
        last_delta = delta
        fps_tracker.tick()

    evaluation = evaluator.evaluate(est_poses, gt_poses_local[:len(est_poses)])

    return OdometryRunResult(
        est_poses=est_poses,
        gt_poses=gt_poses_local[:len(est_poses)],
        avg_fps=fps_tracker.average,
        evaluation=evaluation,
    )


def plot_trajectory(
    est_poses: Sequence[np.ndarray],
    gt_poses: Sequence[np.ndarray],
    title: str = "Trajectory",
) -> None:
    est_traj = np.array([pose[:3, 3] for pose in est_poses], dtype=np.float64)
    gt_traj = np.array([pose[:3, 3] for pose in gt_poses], dtype=np.float64)

    pyplot.clf()

    # pyplot.rcParams["font.family"] = "serif"
    # pyplot.rcParams["font.serif"] = ["Times New Roman"]

    pyplot.plot(gt_traj[:, 0], gt_traj[:, 1], label="ground truth")
    pyplot.plot(est_traj[:, 0], est_traj[:, 1], "--", label="voxel-to-voxel GICP")
    
    pyplot.axis("equal")
    # pyplot.title(title)
    pyplot.xlabel("x, m")
    pyplot.ylabel("y, m")
    pyplot.legend()
    pyplot.show()


def evaluate_and_plot(
    reg,
    dataset,
    downsample_resolution: float = 0.5,
    segment_lengths: Sequence[int] = (100, 200, 300, 400, 500, 600, 700, 800),
    plot_title: str = "Trajectory",
) -> OdometryRunResult:
    result = run_lo(
        reg=reg,
        dataset=dataset,
        downsample_resolution=downsample_resolution,
        segment_lengths=segment_lengths,
    )

    print(f"Avg fps: {result.avg_fps:.3f}")
    result.evaluation.print()
    plot_trajectory(result.est_poses, result.gt_poses, title=plot_title)

    return result