#!/usr/bin/env python3

import argparse
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import open3d as o3d


class LevelingError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect the floor plane in a PCD map and rotate the map until the floor is horizontal."
    )
    parser.add_argument("input_pcd", help="Path to the input PCD file")
    parser.add_argument("-o", "--output", help="Path to the output PCD file")
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="Replace the input file after writing a backup copy",
    )
    parser.add_argument(
        "--backup",
        help="Backup path used with --in-place (default: <input>_pre_floor_level.pcd)",
    )
    parser.add_argument(
        "--report",
        help="Optional JSON file that stores the detected floor plane and applied rotation",
    )
    parser.add_argument("--seed", type=int, default=7, help="Random seed for RANSAC")
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=200000,
        help="Maximum number of candidate floor points passed to RANSAC",
    )
    parser.add_argument(
        "--plane-distance",
        type=float,
        default=0.08,
        help="RANSAC inlier threshold in meters for the first pass",
    )
    parser.add_argument(
        "--refine-distance",
        type=float,
        default=0.05,
        help="RANSAC inlier threshold in meters for the refinement pass",
    )
    parser.add_argument(
        "--local-min-z-radius",
        type=float,
        help="Fit the floor from points within this XY radius of the current minimum-Z point",
    )
    args = parser.parse_args()
    if args.output and args.in_place:
        raise SystemExit("--output cannot be combined with --in-place")
    if args.backup and not args.in_place:
        raise SystemExit("--backup can only be used with --in-place")
    return args


def run_command(command: Sequence[str]) -> None:
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        stderr = result.stderr.strip()
        stdout = result.stdout.strip()
        details = stderr or stdout or "command failed"
        raise LevelingError(f"{' '.join(command)}: {details}")


def normalize(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length < 1e-12:
        raise LevelingError("cannot normalize a near-zero vector")
    return vector / length


def rotation_matrix_from_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = normalize(axis)
    x, y, z = axis
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=np.float64,
    )


def rotation_matrix_from_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source = normalize(source)
    target = normalize(target)
    cross = np.cross(source, target)
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    cross_norm = float(np.linalg.norm(cross))

    if cross_norm < 1e-12:
        if dot > 0.0:
            return np.eye(3, dtype=np.float64)
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(source[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        axis = normalize(np.cross(source, axis))
        return rotation_matrix_from_axis_angle(axis, math.pi)

    skew = np.array(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + skew + skew @ skew * ((1.0 - dot) / (cross_norm * cross_norm))


def axis_angle_from_matrix(rotation: np.ndarray) -> Tuple[np.ndarray, float]:
    trace = float(np.trace(rotation))
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5)))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 1.0], dtype=np.float64), 0.0

    denom = 2.0 * math.sin(angle)
    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=np.float64,
    ) / denom
    return normalize(axis), math.degrees(angle)


def euler_xyz_deg(rotation: np.ndarray) -> List[float]:
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-9

    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0

    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def lowest_significant_peak(
    z_values: np.ndarray,
    bin_width: float = 0.05,
    peak_fraction: float = 0.30,
    smooth_half_window: int = 5,
) -> float:
    del peak_fraction
    del smooth_half_window

    finite_z = np.asarray(z_values[np.isfinite(z_values)], dtype=np.float64)
    if finite_z.size == 0:
        raise LevelingError("no finite Z values found")

    z_min = float(finite_z.min())
    z_max = float(finite_z.max())
    z_range = z_max - z_min
    if z_range < 1e-6:
        return float(finite_z.mean())

    # The floor in this map is the lowest surface, so seed the fit from the
    # bottom slice instead of searching the entire height histogram.
    bottom_span = max(6.0 * bin_width, 0.30)
    bottom_cap = z_min + min(bottom_span, z_range)
    bottom_candidates = finite_z[finite_z <= bottom_cap]
    if bottom_candidates.size == 0:
        return z_min

    if bottom_candidates.size < 32:
        return float(np.median(bottom_candidates))

    local_range = float(bottom_candidates.max() - z_min)
    num_bins = max(1, int(math.ceil(local_range / bin_width)) + 1)
    hist, _ = np.histogram(bottom_candidates, bins=num_bins, range=(z_min, z_min + num_bins * bin_width))
    floor_bin = int(np.argmax(hist))
    return z_min + (floor_bin + 0.5) * bin_width


def adaptive_band(z_values: np.ndarray, minimum: float, maximum: float, scale: float) -> float:
    finite_z = z_values[np.isfinite(z_values)]
    if finite_z.size == 0:
        return minimum
    z_range = float(finite_z.max() - finite_z.min())
    return max(minimum, min(maximum, z_range * scale))


def fit_floor_plane(
    candidate_points: np.ndarray,
    distance_threshold: float,
    num_iterations: int,
    max_candidates: int,
    seed: int,
) -> dict:
    if candidate_points.shape[0] < 3:
        raise LevelingError("not enough candidate floor points for RANSAC")

    if hasattr(o3d.utility, "random"):
        o3d.utility.random.seed(seed)

    rng = np.random.default_rng(seed)
    sampled_points = candidate_points
    if candidate_points.shape[0] > max_candidates:
        indices = rng.choice(candidate_points.shape[0], size=max_candidates, replace=False)
        sampled_points = candidate_points[indices]

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(sampled_points)
    plane_model, inlier_indices = point_cloud.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=num_iterations,
    )

    normal = np.asarray(plane_model[:3], dtype=np.float64)
    norm = float(np.linalg.norm(normal))
    if norm < 1e-12:
        raise LevelingError("RANSAC produced an invalid floor normal")

    normal /= norm
    offset = float(plane_model[3]) / norm
    if normal[2] < 0.0:
        normal = -normal
        offset = -offset

    distances = np.abs(candidate_points @ normal + offset)
    refined_inliers = candidate_points[distances <= distance_threshold]
    if refined_inliers.shape[0] < 3:
        refined_inliers = sampled_points[np.asarray(inlier_indices, dtype=np.int64)]

    centroid = refined_inliers.mean(axis=0)
    centered = refined_inliers - centroid
    covariance = centered.T @ centered / refined_inliers.shape[0]
    _, eigenvectors = np.linalg.eigh(covariance)
    normal = eigenvectors[:, 0]
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(normal @ centroid)

    final_distances = np.abs(candidate_points @ normal + offset)
    final_inliers = candidate_points[final_distances <= distance_threshold]
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(normal[2])))))

    return {
        "normal": normal,
        "offset": offset,
        "centroid": centroid,
        "inlier_count": int(final_inliers.shape[0]),
        "sampled_count": int(sampled_points.shape[0]),
        "tilt_deg": tilt_deg,
    }


def fit_floor_from_lowest_points(
    points: np.ndarray,
    quantile: float,
    max_candidates: int,
    seed: int,
) -> dict:
    finite_mask = np.isfinite(points).all(axis=1)
    finite_points = points[finite_mask]
    if finite_points.shape[0] < 3:
        raise LevelingError("point cloud does not contain enough finite points")

    quantile = min(max(quantile, 1e-5), 0.05)
    z_values = finite_points[:, 2]
    cutoff = float(np.quantile(z_values, quantile))
    candidate_points = finite_points[z_values <= cutoff]
    if candidate_points.shape[0] < 3:
        raise LevelingError("failed to isolate enough lowest points for the initial floor fit")

    rng = np.random.default_rng(seed)
    sampled_points = candidate_points
    if candidate_points.shape[0] > max_candidates:
        indices = rng.choice(candidate_points.shape[0], size=max_candidates, replace=False)
        sampled_points = candidate_points[indices]

    centroid = sampled_points.mean(axis=0)
    centered = sampled_points - centroid
    covariance = centered.T @ centered / sampled_points.shape[0]
    _, eigenvectors = np.linalg.eigh(covariance)
    normal = eigenvectors[:, 0]
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(normal @ centroid)

    distances = np.abs(candidate_points @ normal + offset)
    inlier_threshold = float(max(0.03, np.quantile(distances, 0.9)))
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(normal[2])))))

    return {
        "normal": normal,
        "offset": offset,
        "centroid": centroid,
        "floor_z_estimate": cutoff,
        "candidate_count": int(candidate_points.shape[0]),
        "inlier_count": int(np.count_nonzero(distances <= inlier_threshold)),
        "sampled_count": int(sampled_points.shape[0]),
        "band": float(cutoff - float(z_values.min())),
        "distance_threshold": inlier_threshold,
        "tilt_deg": tilt_deg,
    }


def fit_floor_near_min_z_point(
    points: np.ndarray,
    xy_radius: float,
    distance_threshold: float,
    max_candidates: int,
    seed: int,
) -> dict:
    if xy_radius <= 0.0:
        raise LevelingError("--local-min-z-radius must be positive")

    finite_mask = np.isfinite(points).all(axis=1)
    finite_points = points[finite_mask]
    if finite_points.shape[0] < 3:
        raise LevelingError("point cloud does not contain enough finite points")

    min_point = finite_points[np.argmin(finite_points[:, 2])]
    xy_distances = np.linalg.norm(finite_points[:, :2] - min_point[:2], axis=1)
    candidate_points = finite_points[xy_distances <= xy_radius]
    if candidate_points.shape[0] < 3:
        raise LevelingError("failed to isolate enough points near the minimum-Z floor patch")

    fit = fit_floor_plane(
        candidate_points=candidate_points,
        distance_threshold=distance_threshold,
        num_iterations=5000,
        max_candidates=max_candidates,
        seed=seed,
    )
    fit.update(
        {
            "floor_z_estimate": float(min_point[2]),
            "candidate_count": int(candidate_points.shape[0]),
            "band": float(xy_radius),
            "distance_threshold": float(distance_threshold),
            "min_point": min_point,
            "xy_radius": float(xy_radius),
        }
    )
    return fit


def analyze_floor(
    points: np.ndarray,
    band: float,
    distance_threshold: float,
    max_candidates: int,
    seed: int,
) -> dict:
    finite_mask = np.isfinite(points).all(axis=1)
    finite_points = points[finite_mask]
    if finite_points.shape[0] < 3:
        raise LevelingError("point cloud does not contain enough finite points")

    floor_z_estimate = lowest_significant_peak(finite_points[:, 2])
    lower_slack = max(distance_threshold * 2.0, 0.05)
    band_mask = (finite_points[:, 2] >= floor_z_estimate - lower_slack) & (
        finite_points[:, 2] <= floor_z_estimate + band
    )
    candidate_points = finite_points[band_mask]
    if candidate_points.shape[0] < 3:
        raise LevelingError("failed to isolate enough floor candidates near the lowest dense Z band")

    fit = fit_floor_plane(
        candidate_points=candidate_points,
        distance_threshold=distance_threshold,
        num_iterations=4000,
        max_candidates=max_candidates,
        seed=seed,
    )
    fit.update(
        {
            "floor_z_estimate": float(floor_z_estimate),
            "candidate_count": int(candidate_points.shape[0]),
            "band": float(band),
            "distance_threshold": float(distance_threshold),
        }
    )
    return fit


def select_best_floor_estimate(
    points: np.ndarray,
    max_candidates: int,
    seed: int,
    distance_threshold: float,
) -> dict:
    lowest_floor = fit_floor_from_lowest_points(
        points,
        quantile=0.005,
        max_candidates=max_candidates,
        seed=seed,
    )

    refine_band = adaptive_band(points[:, 2], minimum=0.20, maximum=1.0, scale=0.04)
    try:
        band_floor = analyze_floor(
            points,
            band=refine_band,
            distance_threshold=distance_threshold,
            max_candidates=max_candidates,
            seed=seed + 1,
        )
    except LevelingError:
        return lowest_floor

    allowed_tilt = max(lowest_floor["tilt_deg"] + 2.0, lowest_floor["tilt_deg"] * 2.0)
    if band_floor["tilt_deg"] < lowest_floor["tilt_deg"] and band_floor["tilt_deg"] <= allowed_tilt:
        return band_floor

    return lowest_floor


def serialize_floor_report(report: dict) -> dict:
    serialized = {
        "normal": [float(value) for value in report["normal"]],
        "offset": float(report["offset"]),
        "centroid": [float(value) for value in report["centroid"]],
        "floor_z_estimate": float(report["floor_z_estimate"]),
        "candidate_count": int(report["candidate_count"]),
        "inlier_count": int(report["inlier_count"]),
        "sampled_count": int(report["sampled_count"]),
        "band": float(report["band"]),
        "distance_threshold": float(report["distance_threshold"]),
        "tilt_deg": float(report["tilt_deg"]),
    }
    if "min_point" in report:
        serialized["min_point"] = [float(value) for value in report["min_point"]]
    if "xy_radius" in report:
        serialized["xy_radius"] = float(report["xy_radius"])
    return serialized


def estimate_leveling_rotation(
    points: np.ndarray,
    max_candidates: int,
    seed: int,
    plane_distance: float,
    refine_distance: float,
    local_min_z_radius: Optional[float] = None,
) -> dict:
    rotation_total = np.eye(3, dtype=np.float64)
    pass_reports = []

    if local_min_z_radius is not None:
        initial_floor = fit_floor_near_min_z_point(
            points,
            xy_radius=local_min_z_radius,
            distance_threshold=plane_distance,
            max_candidates=max_candidates,
            seed=seed,
        )
    else:
        initial_floor = fit_floor_from_lowest_points(
            points,
            quantile=0.005,
            max_candidates=max_candidates,
            seed=seed,
        )
    pass_reports.append(initial_floor)

    rotation_step = rotation_matrix_from_vectors(initial_floor["normal"], np.array([0.0, 0.0, 1.0]))
    rotation_total = rotation_step @ rotation_total
    transformed_points = points @ rotation_total.T

    if local_min_z_radius is not None:
        refined_floor = fit_floor_near_min_z_point(
            transformed_points,
            xy_radius=local_min_z_radius,
            distance_threshold=refine_distance,
            max_candidates=max_candidates,
            seed=seed + 1,
        )
    else:
        refined_floor = select_best_floor_estimate(
            transformed_points,
            distance_threshold=refine_distance,
            max_candidates=max_candidates,
            seed=seed + 1,
        )
    pass_reports.append(refined_floor)

    if refined_floor["tilt_deg"] > 1e-3:
        refine_step = rotation_matrix_from_vectors(refined_floor["normal"], np.array([0.0, 0.0, 1.0]))
        rotation_total = refine_step @ rotation_total
        transformed_points = points @ rotation_total.T

    if local_min_z_radius is not None:
        final_floor = fit_floor_near_min_z_point(
            transformed_points,
            xy_radius=local_min_z_radius,
            distance_threshold=refine_distance,
            max_candidates=max_candidates,
            seed=seed + 2,
        )
    else:
        final_floor = select_best_floor_estimate(
            transformed_points,
            distance_threshold=refine_distance,
            max_candidates=max_candidates,
            seed=seed + 2,
        )

    axis, angle_deg = axis_angle_from_matrix(rotation_total)
    return {
        "rotation": rotation_total,
        "initial_floor": initial_floor,
        "refined_floor": refined_floor,
        "final_floor": final_floor,
        "axis": axis,
        "angle_deg": angle_deg,
        "euler_xyz_deg": euler_xyz_deg(rotation_total),
        "passes": [serialize_floor_report(report) for report in pass_reports],
    }


def parse_ascii_header(ascii_pcd: Path) -> Tuple[List[str], List[str], int]:
    header_lines = []
    data_start = None
    with ascii_pcd.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle):
            header_lines.append(line)
            if line.strip().upper().startswith("DATA "):
                data_start = line_number + 1
                break

    if data_start is None:
        raise LevelingError(f"{ascii_pcd} does not contain a DATA header")

    fields = None
    for line in header_lines:
        parts = line.strip().split()
        if parts and parts[0].upper() == "FIELDS":
            fields = parts[1:]
            break

    if not fields:
        raise LevelingError(f"{ascii_pcd} does not contain a FIELDS header")

    return header_lines, fields, data_start


def transform_ascii_pcd(input_ascii: Path, output_ascii: Path, rotation: np.ndarray) -> None:
    header_lines, fields, data_start = parse_ascii_header(input_ascii)
    data = np.loadtxt(input_ascii, skiprows=data_start, dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    if data.shape[1] != len(fields):
        raise LevelingError(
            f"field count mismatch in {input_ascii}: expected {len(fields)} columns, found {data.shape[1]}"
        )

    xyz_indices = [fields.index(name) for name in ("x", "y", "z")]
    xyz = data[:, xyz_indices]
    xyz_mask = np.isfinite(xyz).all(axis=1)
    xyz[xyz_mask] = xyz[xyz_mask] @ rotation.T
    data[:, xyz_indices] = xyz

    if all(name in fields for name in ("normal_x", "normal_y", "normal_z")):
        normal_indices = [fields.index(name) for name in ("normal_x", "normal_y", "normal_z")]
        normals = data[:, normal_indices]
        normal_mask = np.isfinite(normals).all(axis=1)
        normals[normal_mask] = normals[normal_mask] @ rotation.T
        data[:, normal_indices] = normals

    with output_ascii.open("w", encoding="utf-8") as handle:
        handle.writelines(header_lines)
        np.savetxt(handle, data, fmt="%.9g")


def load_points_for_analysis(input_pcd: Path) -> np.ndarray:
    point_cloud = o3d.io.read_point_cloud(
        str(input_pcd),
        remove_nan_points=True,
        remove_infinite_points=True,
    )
    points = np.asarray(point_cloud.points, dtype=np.float64)
    if points.size == 0:
        raise LevelingError(f"{input_pcd} does not contain any readable points")
    return points


def default_output_path(input_pcd: Path, in_place: bool) -> Path:
    if in_place:
        return input_pcd
    return input_pcd.with_name(f"{input_pcd.stem}_leveled{input_pcd.suffix}")


def default_backup_path(input_pcd: Path) -> Path:
    return input_pcd.with_name(f"{input_pcd.stem}_pre_floor_level{input_pcd.suffix}")


def verify_output(
    output_pcd: Path,
    max_candidates: int,
    seed: int,
    refine_distance: float,
    local_min_z_radius: Optional[float] = None,
) -> dict:
    points = load_points_for_analysis(output_pcd)
    if local_min_z_radius is not None:
        return fit_floor_near_min_z_point(
            points,
            xy_radius=local_min_z_radius,
            distance_threshold=refine_distance,
            max_candidates=max_candidates,
            seed=seed,
        )
    verify_band = adaptive_band(points[:, 2], minimum=0.20, maximum=1.0, scale=0.04)
    return analyze_floor(
        points,
        band=verify_band,
        distance_threshold=refine_distance,
        max_candidates=max_candidates,
        seed=seed,
    )


def build_report(
    input_pcd: Path,
    output_pcd: Path,
    backup_pcd: Optional[Path],
    point_count: int,
    estimation: dict,
    verification: dict,
) -> dict:
    rotation = estimation["rotation"]
    matrix_4x4 = np.eye(4, dtype=np.float64)
    matrix_4x4[:3, :3] = rotation

    return {
        "input_pcd": str(input_pcd),
        "output_pcd": str(output_pcd),
        "backup_pcd": str(backup_pcd) if backup_pcd else None,
        "point_count": int(point_count),
        "rotation_matrix_3x3": rotation.tolist(),
        "transform_matrix_4x4": matrix_4x4.tolist(),
        "rotation_axis": [float(value) for value in estimation["axis"]],
        "rotation_angle_deg": float(estimation["angle_deg"]),
        "rotation_euler_xyz_deg": [float(value) for value in estimation["euler_xyz_deg"]],
        "initial_floor": serialize_floor_report(estimation["initial_floor"]),
        "refined_floor": serialize_floor_report(estimation["refined_floor"]),
        "final_floor": serialize_floor_report(estimation["final_floor"]),
        "output_verification": serialize_floor_report(verification),
        "passes": estimation["passes"],
    }


def write_report(report_path: Path, report: dict) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    input_pcd = Path(args.input_pcd).expanduser().resolve()
    if not input_pcd.is_file():
        raise SystemExit(f"input PCD not found: {input_pcd}")

    output_pcd = Path(args.output).expanduser().resolve() if args.output else default_output_path(input_pcd, args.in_place)
    backup_pcd = None
    if args.in_place:
        backup_pcd = Path(args.backup).expanduser().resolve() if args.backup else default_backup_path(input_pcd)
        if backup_pcd == input_pcd:
            raise SystemExit("backup path must differ from the input path")
        if backup_pcd.exists():
            raise SystemExit(f"backup path already exists: {backup_pcd}")

    points = load_points_for_analysis(input_pcd)
    estimation = estimate_leveling_rotation(
        points=points,
        max_candidates=args.max_candidates,
        seed=args.seed,
        plane_distance=args.plane_distance,
        refine_distance=args.refine_distance,
        local_min_z_radius=args.local_min_z_radius,
    )

    with tempfile.TemporaryDirectory(prefix="pcd_floor_level_") as temp_dir:
        temp_dir_path = Path(temp_dir)
        ascii_input = temp_dir_path / "input_ascii.pcd"
        ascii_output = temp_dir_path / "output_ascii.pcd"
        binary_output = temp_dir_path / "output_binary.pcd"

        run_command(["pcl_convert_pcd_ascii_binary", str(input_pcd), str(ascii_input), "0"])
        transform_ascii_pcd(ascii_input, ascii_output, estimation["rotation"])
        run_command(["pcl_convert_pcd_ascii_binary", str(ascii_output), str(binary_output), "1"])

        output_pcd.parent.mkdir(parents=True, exist_ok=True)
        if args.in_place:
            shutil.copy2(input_pcd, backup_pcd)
        binary_output.replace(output_pcd)

    verification = verify_output(
        output_pcd=output_pcd,
        max_candidates=args.max_candidates,
        seed=args.seed + 3,
        refine_distance=args.refine_distance,
        local_min_z_radius=args.local_min_z_radius,
    )
    report = build_report(
        input_pcd=input_pcd,
        output_pcd=output_pcd,
        backup_pcd=backup_pcd,
        point_count=points.shape[0],
        estimation=estimation,
        verification=verification,
    )

    if args.report:
        write_report(Path(args.report).expanduser().resolve(), report)

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except LevelingError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)