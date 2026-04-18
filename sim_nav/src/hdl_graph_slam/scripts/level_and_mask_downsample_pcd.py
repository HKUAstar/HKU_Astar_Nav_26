#!/usr/bin/env python3

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from level_pcd_floor import LevelingError, estimate_leveling_rotation, load_points_for_analysis, serialize_floor_report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Level a binary PCD map by the detected floor plane, then voxel-downsample only points "
            "with xy radius above a threshold and z below a threshold."
        )
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
        help="Backup path used with --in-place (default: <input>_pre_level_mask_downsample.pcd)",
    )
    parser.add_argument(
        "--report",
        help="Optional JSON file storing the applied leveling transform and selective downsampling statistics",
    )
    parser.add_argument("--seed", type=int, default=7, help="Random seed for floor fitting")
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=200000,
        help="Maximum number of candidate floor points passed to plane fitting",
    )
    parser.add_argument(
        "--plane-distance",
        type=float,
        default=0.08,
        help="Initial RANSAC inlier threshold in meters",
    )
    parser.add_argument(
        "--refine-distance",
        type=float,
        default=0.05,
        help="Refinement inlier threshold in meters",
    )
    parser.add_argument(
        "--xy-radius-threshold",
        type=float,
        default=20.0,
        help="Only points with sqrt(x^2+y^2) >= this threshold are eligible for voxel downsampling",
    )
    parser.add_argument(
        "--z-threshold",
        type=float,
        default=9.0,
        help="Only points with z <= this threshold are eligible for voxel downsampling",
    )
    parser.add_argument(
        "--leaf-size",
        type=float,
        default=0.2,
        help="Voxel leaf size in meters applied to the eligible subset",
    )
    args = parser.parse_args()
    if args.output and args.in_place:
        raise SystemExit("--output cannot be combined with --in-place")
    if args.backup and not args.in_place:
        raise SystemExit("--backup can only be used with --in-place")
    return args


def default_output_path(input_pcd: Path, in_place: bool) -> Path:
    if in_place:
        return input_pcd
    return input_pcd.with_name(f"{input_pcd.stem}_leveled_masked_downsampled{input_pcd.suffix}")


def default_backup_path(input_pcd: Path) -> Path:
    return input_pcd.with_name(f"{input_pcd.stem}_pre_level_mask_downsample{input_pcd.suffix}")


def parse_pcd_header(input_pcd: Path) -> Tuple[List[str], Dict[str, str], int]:
    header_lines: List[str] = []
    byte_count = 0
    with input_pcd.open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                break
            byte_count += len(line)
            decoded = line.decode("ascii")
            header_lines.append(decoded)
            if decoded.strip().upper().startswith("DATA "):
                break

    if not header_lines or not header_lines[-1].strip().upper().startswith("DATA "):
        raise LevelingError(f"{input_pcd} does not contain a valid PCD header")

    header_map: Dict[str, str] = {}
    for line in header_lines:
        parts = line.strip().split(maxsplit=1)
        if not parts:
            continue
        key = parts[0].upper()
        value = parts[1] if len(parts) > 1 else ""
        header_map[key] = value

    return header_lines, header_map, byte_count


def load_binary_pcd_matrix(input_pcd: Path) -> Tuple[List[str], List[str], np.ndarray]:
    header_lines, header_map, data_offset = parse_pcd_header(input_pcd)
    if header_map.get("DATA", "").lower() != "binary":
        raise LevelingError(f"{input_pcd} uses DATA {header_map.get('DATA', '')}, only binary PCD is supported")

    fields = header_map.get("FIELDS", "").split()
    sizes = [int(value) for value in header_map.get("SIZE", "").split()]
    types = header_map.get("TYPE", "").split()
    counts = [int(value) for value in header_map.get("COUNT", "").split()] if "COUNT" in header_map else [1] * len(fields)
    points = int(header_map.get("POINTS", "0"))

    if not fields or not sizes or not types or not counts:
        raise LevelingError(f"{input_pcd} is missing FIELDS/SIZE/TYPE/COUNT metadata")
    if len(fields) != len(sizes) or len(fields) != len(types) or len(fields) != len(counts):
        raise LevelingError(f"{input_pcd} has inconsistent field metadata")
    if any(size != 4 for size in sizes) or any(value != "F" for value in types) or any(count != 1 for count in counts):
        raise LevelingError(
            f"{input_pcd} must use float32 scalar fields for binary processing; found SIZE={sizes}, TYPE={types}, COUNT={counts}"
        )

    with input_pcd.open("rb") as handle:
        handle.seek(data_offset)
        data = np.fromfile(handle, dtype=np.float32)

    column_count = len(fields)
    expected_values = points * column_count
    if data.size != expected_values:
        raise LevelingError(
            f"{input_pcd} payload size mismatch: expected {expected_values} float32 values, found {data.size}"
        )

    return header_lines, fields, data.reshape(points, column_count)


def update_header_counts(header_lines: Sequence[str], point_count: int) -> List[str]:
    updated: List[str] = []
    for line in header_lines:
        stripped = line.strip()
        if stripped.upper().startswith("WIDTH "):
            updated.append(f"WIDTH {point_count}\n")
        elif stripped.upper().startswith("HEIGHT "):
            updated.append("HEIGHT 1\n")
        elif stripped.upper().startswith("POINTS "):
            updated.append(f"POINTS {point_count}\n")
        else:
            updated.append(line)
    return updated


def rotate_points_and_normals(matrix: np.ndarray, fields: Sequence[str], rotation: np.ndarray) -> np.ndarray:
    rotated = matrix.copy()
    xyz_indices = [fields.index(name) for name in ("x", "y", "z")]
    xyz = rotated[:, xyz_indices]
    xyz_mask = np.isfinite(xyz).all(axis=1)
    xyz[xyz_mask] = xyz[xyz_mask] @ rotation.T
    rotated[:, xyz_indices] = xyz

    if all(name in fields for name in ("normal_x", "normal_y", "normal_z")):
        normal_indices = [fields.index(name) for name in ("normal_x", "normal_y", "normal_z")]
        normals = rotated[:, normal_indices]
        normal_mask = np.isfinite(normals).all(axis=1)
        normals[normal_mask] = normals[normal_mask] @ rotation.T
        lengths = np.linalg.norm(normals[normal_mask], axis=1)
        nonzero = lengths > 1e-12
        normals_subset = normals[normal_mask]
        normals_subset[nonzero] = normals_subset[nonzero] / lengths[nonzero, None]
        normals[normal_mask] = normals_subset
        rotated[:, normal_indices] = normals

    return rotated


def voxel_downsample_subset(subset: np.ndarray, fields: Sequence[str], leaf_size: float) -> np.ndarray:
    if subset.shape[0] == 0:
        return subset
    if leaf_size <= 0.0:
        raise LevelingError("leaf size must be positive")

    xyz_indices = [fields.index(name) for name in ("x", "y", "z")]
    xyz = subset[:, xyz_indices]
    voxel_indices = np.floor(xyz / leaf_size).astype(np.int64)

    order = np.lexsort((voxel_indices[:, 2], voxel_indices[:, 1], voxel_indices[:, 0]))
    sorted_voxels = voxel_indices[order]
    sorted_subset = subset[order]

    boundaries = np.empty(sorted_subset.shape[0], dtype=bool)
    boundaries[0] = True
    boundaries[1:] = np.any(sorted_voxels[1:] != sorted_voxels[:-1], axis=1)
    group_starts = np.flatnonzero(boundaries)

    sums = np.add.reduceat(sorted_subset.astype(np.float64), group_starts, axis=0)
    counts = np.diff(np.append(group_starts, sorted_subset.shape[0])).astype(np.float64)
    reduced = (sums / counts[:, None]).astype(np.float32)

    if all(name in fields for name in ("normal_x", "normal_y", "normal_z")):
        normal_indices = [fields.index(name) for name in ("normal_x", "normal_y", "normal_z")]
        normals = reduced[:, normal_indices]
        lengths = np.linalg.norm(normals, axis=1)
        valid = lengths > 1e-12
        normals[valid] = normals[valid] / lengths[valid, None]
        reduced[:, normal_indices] = normals

    return reduced


def selective_downsample(
    matrix: np.ndarray,
    fields: Sequence[str],
    xy_radius_threshold: float,
    z_threshold: float,
    leaf_size: float,
) -> Tuple[np.ndarray, Dict[str, int]]:
    xyz_indices = [fields.index(name) for name in ("x", "y", "z")]
    xyz = matrix[:, xyz_indices]
    finite_mask = np.isfinite(xyz).all(axis=1)
    xy_radius = np.hypot(xyz[:, 0], xyz[:, 1])
    eligible_mask = finite_mask & (xy_radius >= xy_radius_threshold) & (xyz[:, 2] <= z_threshold)

    untouched = matrix[~eligible_mask]
    eligible = matrix[eligible_mask]
    reduced = voxel_downsample_subset(eligible, fields, leaf_size)
    output = np.concatenate([untouched, reduced], axis=0) if reduced.size else untouched.copy()

    stats = {
        "input_point_count": int(matrix.shape[0]),
        "eligible_point_count": int(eligible.shape[0]),
        "untouched_point_count": int(untouched.shape[0]),
        "downsampled_eligible_point_count": int(reduced.shape[0]),
        "output_point_count": int(output.shape[0]),
        "removed_point_count": int(eligible.shape[0] - reduced.shape[0]),
    }
    return output, stats


def write_binary_pcd(output_pcd: Path, header_lines: Sequence[str], matrix: np.ndarray) -> None:
    updated_header = update_header_counts(header_lines, int(matrix.shape[0]))
    output_pcd.parent.mkdir(parents=True, exist_ok=True)
    with output_pcd.open("wb") as handle:
        for line in updated_header:
            handle.write(line.encode("ascii"))
        matrix.astype(np.float32, copy=False).tofile(handle)


def build_report(
    input_pcd: Path,
    output_pcd: Path,
    backup_pcd: Optional[Path],
    estimation: dict,
    downsample_stats: Dict[str, int],
    xy_radius_threshold: float,
    z_threshold: float,
    leaf_size: float,
) -> dict:
    rotation = estimation["rotation"]
    matrix_4x4 = np.eye(4, dtype=np.float64)
    matrix_4x4[:3, :3] = rotation

    return {
        "input_pcd": str(input_pcd),
        "output_pcd": str(output_pcd),
        "backup_pcd": str(backup_pcd) if backup_pcd else None,
        "rotation_matrix_3x3": rotation.tolist(),
        "transform_matrix_4x4": matrix_4x4.tolist(),
        "rotation_axis": [float(value) for value in estimation["axis"]],
        "rotation_angle_deg": float(estimation["angle_deg"]),
        "rotation_euler_xyz_deg": [float(value) for value in estimation["euler_xyz_deg"]],
        "initial_floor": serialize_floor_report(estimation["initial_floor"]),
        "refined_floor": serialize_floor_report(estimation["refined_floor"]),
        "final_floor": serialize_floor_report(estimation["final_floor"]),
        "passes": estimation["passes"],
        "xy_radius_threshold": float(xy_radius_threshold),
        "z_threshold": float(z_threshold),
        "leaf_size": float(leaf_size),
        "downsample_stats": downsample_stats,
    }


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
    )

    header_lines, fields, matrix = load_binary_pcd_matrix(input_pcd)
    rotated_matrix = rotate_points_and_normals(matrix, fields, estimation["rotation"])
    output_matrix, downsample_stats = selective_downsample(
        rotated_matrix,
        fields,
        xy_radius_threshold=args.xy_radius_threshold,
        z_threshold=args.z_threshold,
        leaf_size=args.leaf_size,
    )

    if args.in_place:
        shutil.copy2(input_pcd, backup_pcd)
    write_binary_pcd(output_pcd, header_lines, output_matrix)

    report = build_report(
        input_pcd=input_pcd,
        output_pcd=output_pcd,
        backup_pcd=backup_pcd,
        estimation=estimation,
        downsample_stats=downsample_stats,
        xy_radius_threshold=args.xy_radius_threshold,
        z_threshold=args.z_threshold,
        leaf_size=args.leaf_size,
    )
    if args.report:
        Path(args.report).expanduser().resolve().write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except LevelingError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)