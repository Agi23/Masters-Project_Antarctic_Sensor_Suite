# Software_Sync

Single-command software-synchronized capture for a Livox LiDAR + stereo camera pair.
Does not depend on `src/Hardware_Sync`.

## Quick Start

```bash
cd src/Software_Sync
./run_capture.sh --frames 100
```

---

## Recording in LVX Format

The LVX format stores all frames in a single binary file with embedded device metadata and timestamps, making it more compact and self-describing than per-frame PCD files.

### Command

```bash
./run_capture.sh \
  --frames 200 \
  --lidar-format lvx \
  --lidar-code "000000000000001"
```

| Flag | Default | Description |
|---|---|---|
| `--frames` | required | Number of synchronized frames to capture |
| `--lidar-format` | `pcd` | `lvx` or `pcd` |
| `--lidar-code` | auto-detect | Livox broadcast code (serial number) of the LiDAR |
| `--camera-timeout-sec` | `0.35` | Per-camera wait timeout after software trigger |
| `--lidar-binary` | auto-detect | Path to `lidar_lvx_sample` binary |
| `--lidar-only` | off | Skip stereo cameras; record LiDAR only |
| `--lidar-param` | off | Load extrinsic parameters from `extrinsic.xml` |
| `--lidar-log` | off | Save Livox SDK internal log |
| `--show-video` | off | Show live camera previews |

### LiDAR binary

The pre-built binary is at `lidar/bin/lidar_lvx_sample`. It is auto-detected if present.
You can override it:

```bash
./run_capture.sh --lidar-binary /absolute/path/to/lidar_lvx_sample --lidar-format lvx --frames 100
```

### LiDAR-only mode (no cameras)

```bash
./run_capture.sh --frames 100 --lidar-format lvx --lidar-only
```

---

## Expected Data Output

Each run creates a timestamped session folder under `captures/`:

```
captures/session_YYYYMMDD_HHMMSS/
│
├── lidar/
│   ├── lvx/
│   │   └── lidar_capture.lvx        # single LVX file containing all frames
│   ├── frames/                      # empty in lvx mode (used for pcd mode)
│   └── raw/                         # working directory for the LiDAR process
│
├── stereo/
│   ├── left/
│   │   └── left_000001_<ns>.png     # one PNG per synchronized frame
│   └── right/
│       └── right_000001_<ns>.png
│
├── metadata/
│   ├── frame_metadata.csv           # per-frame timing and file paths
│   └── session_summary.json         # run parameters and status
│
└── logs/
    └── lidar_stdout.log             # raw stdout from lidar_lvx_sample
```

### LVX file

`lidar/lvx/lidar_capture.lvx` is a Livox proprietary binary format containing:
- A file header with SDK version and device count
- Per-device metadata: broadcast code, extrinsic calibration (roll/pitch/yaw, x/y/z)
- Frame blocks: each block contains one frame's worth of point packets at 20 Hz

The file can be opened in **Livox Viewer** or converted to PCD/CSV using the Livox SDK tools.

### frame_metadata.csv

One row per synchronized frame:

| Column | Description |
|---|---|
| `frame_index` | 1-based frame counter |
| `trigger_time_ns` | Unix timestamp (ns) when software trigger was sent to cameras |
| `lidar_event_time_ns` | Unix timestamp (ns) when the LiDAR frame-complete event was received |
| `lidar_frame_raw_index` | Raw frame index reported by `lidar_lvx_sample` |
| `lidar_file` | Relative path to the LVX file (same for all rows in lvx mode) |
| `left_serial` | Camera serial number |
| `left_capture_time_ns` | Timestamp of left image capture (ns) |
| `left_file` | Relative path to left PNG |
| `right_serial` | Camera serial number |
| `right_capture_time_ns` | Timestamp of right image capture (ns) |
| `right_file` | Relative path to right PNG |

### session_summary.json

Records the full run configuration: status (`ok` / `failed`), start/end UTC timestamps,
frames requested vs captured, LiDAR binary path, format, command used, and paths to
the metadata CSV and log file.

---

## Viewing a Session

```bash
python3 data_viewer/view_sync_session.py \
  --session-dir captures/session_YYYYMMDD_HHMMSS
```

Controls: `Left/Right` step one frame, `Up/Down` jump 10, `Home/End` first/last, `Q/Esc` quit.

When `metadata/frame_metadata.csv` exists the viewer shows per-frame timestamp deltas
relative to the LiDAR event time.

---

## Config

Default stereo camera config: `config/stereo_software_trigger.json`

Edit this file to change camera serials, resolution, pixel format, exposure, or trigger property names.

---

## Folder Layout

```
src/Software_Sync/
├── run_capture.sh                   # entry point
├── scripts/
│   └── software_sync_capture_new.py # capture orchestrator
├── lidar/
│   ├── bin/
│   │   └── lidar_lvx_sample         # pre-built LiDAR binary
│   └── src/
│       └── main.cpp                 # modified Livox SDK sample source
├── data_viewer/
│   └── view_sync_session.py         # interactive session viewer
├── config/
│   └── stereo_software_trigger.json # camera configuration
└── captures/                        # session output (gitignored)
```
