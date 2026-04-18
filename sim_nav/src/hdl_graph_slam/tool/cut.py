import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

input_pcd = Path('/home/sentry/AstarTraining/Old_nav/sim_nav/src/hdl_graph_slam/map/stadium_Moscow.pcd')
backup_pcd = input_pcd.with_name('stadium_Moscow_pre_cutoff_filter_v4.pcd')
report_path = input_pcd.with_name('stadium_Moscow_cutoff_filter_v4_report.json')

if backup_pcd.exists():
    raise SystemExit(f'backup already exists: {backup_pcd}')


def run(cmd):
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or result.stdout.strip() or 'command failed')

with tempfile.TemporaryDirectory(prefix='pcd_cutoff_filter_v4_') as temp_dir:
    temp_dir = Path(temp_dir)
    ascii_input = temp_dir / 'input_ascii.pcd'
    ascii_output = temp_dir / 'output_ascii.pcd'
    binary_output = temp_dir / 'output_binary.pcd'

    run(['pcl_convert_pcd_ascii_binary', str(input_pcd), str(ascii_input), '0'])

    with ascii_input.open('r', encoding='utf-8') as handle:
        header_lines = []
        data_start = None
        for line_number, line in enumerate(handle):
            header_lines.append(line)
            if line.strip().upper().startswith('DATA '):
                data_start = line_number + 1
                break
    if data_start is None:
        raise SystemExit('missing DATA header')

    fields = None
    for line in header_lines:
        parts = line.strip().split()
        if parts and parts[0].upper() == 'FIELDS':
            fields = parts[1:]
            break
    if not fields:
        raise SystemExit('missing FIELDS header')

    data = np.loadtxt(ascii_input, skiprows=data_start, dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    x_index = fields.index('x')
    y_index = fields.index('y')
    z_index = fields.index('z')

    x = data[:, x_index]
    y = data[:, y_index]
    z = data[:, z_index]

    remove_mask = (((x >= 15.0) | (y >= 15.0)) & (z <= 10.0)) | (np.abs(y) > 30.0)
    kept = data[~remove_mask]

    updated_header = []
    for line in header_lines:
        stripped = line.strip()
        if stripped.upper().startswith('WIDTH '):
            updated_header.append(f'WIDTH {kept.shape[0]}\n')
        elif stripped.upper().startswith('HEIGHT '):
            updated_header.append('HEIGHT 1\n')
        elif stripped.upper().startswith('POINTS '):
            updated_header.append(f'POINTS {kept.shape[0]}\n')
        else:
            updated_header.append(line)

    with ascii_output.open('w', encoding='utf-8') as handle:
        handle.writelines(updated_header)
        np.savetxt(handle, kept, fmt='%.9g')

    run(['pcl_convert_pcd_ascii_binary', str(ascii_output), str(binary_output), '1'])
    shutil.copy2(input_pcd, backup_pcd)
    binary_output.replace(input_pcd)

report = {
    'input_pcd': str(input_pcd),
    'backup_pcd': str(backup_pcd),
    'cutoff_condition': '((x >= 15.0) or (y >= 15.0)) and (z <= 10.0) or (abs(y) > 30.0)',
    'points_before': int(data.shape[0]),
    'points_removed': int(remove_mask.sum()),
    'points_after': int(kept.shape[0]),
}
report_path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
print(json.dumps(report, indent=2))