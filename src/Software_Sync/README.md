# Software_Sync

This folder provides a single-command workflow for software-synchronized Livox LiDAR + stereo camera capture.
It is self-contained and does not depend on `src/Hardware_Sync`.

## What It Does

1. Starts two stereo cameras in software-trigger mode.
2. Starts Livox LiDAR capture in either `pcd` or `lvx` mode.
3. For each saved LiDAR frame event, immediately sends software trigger commands to both cameras.
4. Saves one LiDAR frame and one left/right stereo pair per synchronized index.
5. Stops automatically when the requested number of synchronized frames is reached.

## One Command

From this folder:

```bash
./run_capture.sh --frames 100
```

Optional common flags:

```bash
./run_capture.sh \
  --frames 200 \
  --lidar-format lvx \
  --lidar-code "000000000000001" \
  --camera-timeout-sec 0.4
```

## Folder Layout Per Session

Each run creates:

```text
captures/session_YYYYMMDD_HHMMSS/
  lidar/
    frames/      # renamed LiDAR frames: lidar_000001.pcd ...
    lvx/         # lidar_capture.lvx in lvx mode
    raw/         # raw LiDAR-generated files (remaining/unmapped)
  stereo/
    left/        # left_000001_<timestamp>.png ...
    right/       # right_000001_<timestamp>.png ...
  metadata/
    frame_metadata.csv
    session_summary.json
  logs/
    lidar_stdout.log
```

## View Synchronization

Use the interactive viewer to inspect LiDAR + stereo alignment:

```bash
python3 data_viewer/view_sync_session.py \
  --session-dir captures/session_YYYYMMDD_HHMMSS
```

Controls: `Left/Right` frame step, `Up/Down` jump 10, `Home/End` first/last, `Q/Esc` quit.

When `metadata/frame_metadata.csv` exists, the viewer shows per-frame timestamp deltas relative to LiDAR event time.

## Config

Default camera + trigger config:

- `config/stereo_software_trigger.json`

Edit this file if you need to change serials, pixel format, resolution, exposure, or trigger property names.

## Notes

- Default LiDAR binary path:
  - `src/Software_Sync/lidar/bin/lidar_lvx_sample`
- Local LiDAR sample source copy:
  - `src/Software_Sync/lidar/src/`
- You can override with:
  - `--lidar-binary /absolute/path/to/lidar_lvx_sample`
