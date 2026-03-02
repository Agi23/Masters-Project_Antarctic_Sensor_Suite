# Data Viewer

Interactive viewer for checking frame-by-frame synchronization between:

- LiDAR scans (`lidar/frames/lidar_XXXXXX.pcd`)
- Stereo camera images (`stereo/<camera_a>/...` and `stereo/<camera_b>/...`)

The viewer loads one LiDAR frame with two stereo images side-by-side and lets you step through frames using your keyboard.
LiDAR is shown as an interactive 3D point cloud on a black background.

## Run

From `src/Software_Sync`:

```bash
python3 data_viewer/view_sync_session.py \
  --session-dir captures/session_YYYYMMDD_HHMMSS
```

If your camera folder names are not `left` and `right`, set them explicitly:

```bash
python3 data_viewer/view_sync_session.py \
  --session-dir captures/session_YYYYMMDD_HHMMSS \
  --camera-a cam0 \
  --camera-b cam1
```

## Keyboard Controls

- `Right` / `Left`: next / previous frame
- `Up` / `Down`: jump +10 / -10 frames
- `Home` / `End`: first / last frame
- `Q` or `Esc`: quit

## LiDAR 3D Controls

- Mouse drag on LiDAR plot: rotate
- Mouse wheel: zoom
- Matplotlib toolbar `Pan` tool: pan the 3D view

## Timing Display

When `metadata/frame_metadata.csv` is available, the viewer shows:

- `{camera_a}_capture_time_ns - lidar_event_time_ns`
- `{camera_b}_capture_time_ns - lidar_event_time_ns`
- `{camera_b}_capture_time_ns - {camera_a}_capture_time_ns`

If LiDAR event timestamps are not available, LiDAR-based deltas are shown as `N/A`.
