#!/usr/bin/python3
import os
import sys
import time
import numpy
import pygicp
from matplotlib import pyplot

def pose_distance(p1, p2):
    """Euclidean distance between translations of two 4x4 poses."""
    return numpy.linalg.norm(p1[:3, 3] - p2[:3, 3])

def trajectory_distances(poses):
    """Cumulative distances along trajectory."""
    dists = [0.0]
    for i in range(1, len(poses)):
        dists.append(dists[-1] + pose_distance(poses[i - 1], poses[i]))
    return numpy.array(dists, dtype=numpy.float64)

def rotation_error_rad(T_err):
    """Angle of rotation (rad) from a 4x4 error transform."""
    R = T_err[:3, :3]
    tr = numpy.trace(R)
    c = (tr - 1.0) * 0.5
    c = numpy.clip(c, -1.0, 1.0)
    return float(numpy.arccos(c))

def compute_kitti_segment_errors(est_poses, gt_poses, segment_lengths=(100, 200, 300, 400, 500, 600, 700, 800)):
    """
    KITTI odometry-style errors:
      - translational drift: % of segment length
      - rotational drift: degrees per meter
    """
    assert len(est_poses) == len(gt_poses), "est and gt must have same length"
    n = len(gt_poses)
    gt_dists = trajectory_distances(gt_poses)

    results = {L: {"t_perc": [], "r_deg_per_m": []} for L in segment_lengths}

    for i in range(n):
        for L in segment_lengths:
            target_dist = gt_dists[i] + L
            j = numpy.searchsorted(gt_dists, target_dist, side="left")
            if j >= n:
                continue

            T_gt_rel  = numpy.linalg.inv(gt_poses[i]) @ gt_poses[j]
            T_est_rel = numpy.linalg.inv(est_poses[i]) @ est_poses[j]

            T_err = numpy.linalg.inv(T_gt_rel) @ T_est_rel

            t_err = numpy.linalg.norm(T_err[:3, 3])
            r_err = rotation_error_rad(T_err)

            results[L]["t_perc"].append((t_err / L) * 100.0)
            results[L]["r_deg_per_m"].append((r_err / L) * (180.0 / numpy.pi))

    summary = {}
    for L in segment_lengths:
        t_list = numpy.array(results[L]["t_perc"], dtype=numpy.float64)
        r_list = numpy.array(results[L]["r_deg_per_m"], dtype=numpy.float64)
        summary[L] = {
            "count": int(len(t_list)),
            "t_perc_mean": float(t_list.mean()) if len(t_list) else float("nan"),
            "r_deg_per_m_mean": float(r_list.mean()) if len(r_list) else float("nan"),
        }

    all_t = numpy.concatenate([numpy.array(results[L]["t_perc"], dtype=numpy.float64) for L in segment_lengths if len(results[L]["t_perc"])])
    all_r = numpy.concatenate([numpy.array(results[L]["r_deg_per_m"], dtype=numpy.float64) for L in segment_lengths if len(results[L]["r_deg_per_m"])])
    overall = {
        "t_perc_mean": float(all_t.mean()) if all_t.size else float("nan"),
        "r_deg_per_m_mean": float(all_r.mean()) if all_r.size else float("nan"),
        "num_samples": int(all_t.size),
    }

    return summary, overall

def read_calib_file(filepath):
    """Read in a calibration file and parse into a dictionary."""
    data = {}

    with open(filepath, 'r') as f:
        for line in f.readlines():
            try:
                key, value = line.split(':', 1)
            except ValueError:
                key, value = line.split(' ', 1)
            try:
                data[key] = numpy.array([float(x) for x in value.split()])
            except ValueError:
                pass

    return data

def main():
	if len(sys.argv) < 2:
		print('usage: kitti.py /path/to/kitti/sequences/00/velodyne')
		return
	sequence = '10'
	kitti_path = sys.argv[1]
	
	seq_path = os.path.join(kitti_path, 'sequences', sequence, 'velodyne')
	filenames = sorted([seq_path + '/' + x for x in os.listdir(seq_path) if x.endswith('.bin')])
	
	calib_data = {}
	calib_filepath = os.path.join(kitti_path, 'calibration', 'sequences', sequence, 'calib.txt')
	filedata = read_calib_file(calib_filepath)

	P_rect_00 = numpy.reshape(filedata['P0'], (3, 4))
	P_rect_10 = numpy.reshape(filedata['P1'], (3, 4))
	P_rect_20 = numpy.reshape(filedata['P2'], (3, 4))
	P_rect_30 = numpy.reshape(filedata['P3'], (3, 4))

	calib_data['P_rect_00'] = P_rect_00
	calib_data['P_rect_10'] = P_rect_10
	calib_data['P_rect_20'] = P_rect_20
	calib_data['P_rect_30'] = P_rect_30

	T1 = numpy.eye(4)
	T1[0, 3] = P_rect_10[0, 3] / P_rect_10[0, 0]
	T2 = numpy.eye(4)
	T2[0, 3] = P_rect_20[0, 3] / P_rect_20[0, 0]
	T3 = numpy.eye(4)
	T3[0, 3] = P_rect_30[0, 3] / P_rect_30[0, 0]

	calib_data['T_cam0_velo'] = numpy.reshape(filedata['Tr'], (3, 4))
	calib_data['T_cam0_velo'] = numpy.vstack([calib_data['T_cam0_velo'], [0, 0, 0, 1]])

	pose_file = os.path.join(kitti_path, 'poses', sequence + '.txt')
	gt_poses = []
	with open(pose_file, 'r') as f:
		lines = f.readlines()
		for line in lines:
			T_w_cam0 = numpy.fromstring(line, dtype=float, sep=' ')
			T_w_cam0 = T_w_cam0.reshape(3, 4)
			T_w_cam0 = numpy.vstack((T_w_cam0, [0, 0, 0, 1]))
			gt_poses.append(T_w_cam0 @ calib_data['T_cam0_velo'])

	reg = pygicp.FastVGICP()

	reg.set_num_threads(1)
	# reg.set_resolution(1.0)
	# reg.set_max_correspondence_distance(2.0)

	stamps = [time.time()]
	poses = [gt_poses[0]]
	last_delta = numpy.identity(4)

	vis = True
	fps_sum = 0
	for i, filename in enumerate(filenames):
		points = numpy.fromfile(filename, dtype=numpy.float32).reshape(-1, 4)[:, :3]
		points = pygicp.downsample(points, 0.5)

		if i == 0:
			reg.set_input_target(points)
			delta = numpy.identity(4)
		else:
			reg.set_input_source(points)
			delta = reg.align(initial_guess=last_delta)
			reg.swap_source_and_target()
		
		poses.append(poses[-1].dot(delta))
		last_delta = delta

		if i > 0:
			stamps = stamps[-9:] + [time.time()]
			fps = (len(stamps) / (stamps[-1] - stamps[0]))
			fps_sum += fps

		if vis:
			traj = numpy.array([x[:3, 3] for x in poses])
			gt_traj = numpy.array([x[:3, 3] for x in gt_poses])

			if i % 30 == 0:
				pyplot.clf()
				pyplot.plot(traj[:, 0], traj[:, 2])
				pyplot.plot(gt_traj[:, 0], gt_traj[:, 2], '--')
				pyplot.axis('equal')
				pyplot.pause(0.01)
	print(f"Avg fps: {fps_sum/i}")
      
	# --- Evaluation (KITTI-style drift) ---
	est_poses = poses[1:]
	gt_eval_poses = gt_poses

	summary, overall = compute_kitti_segment_errors(est_poses, gt_eval_poses)

	print("\nKITTI drift metrics:")
	print("Length[m] | samples | t_err[%] | r_err[deg/m]")
	for L in sorted(summary.keys()):
		s = summary[L]
		print(f"{L:8d} | {s['count']:7d} | {s['t_perc_mean']:7.3f} | {s['r_deg_per_m_mean']:10.6f}")

	print(f"\nOverall: t_err[%]={overall['t_perc_mean']:.3f}, r_err[deg/m]={overall['r_deg_per_m_mean']:.6f}, samples={overall['num_samples']}")

if __name__ == '__main__':
	main()
