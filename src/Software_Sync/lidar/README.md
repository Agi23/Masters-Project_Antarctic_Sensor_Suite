# LiDAR (Software_Sync Copy)

This directory is a local copy of the modified Livox `lidar_lvx_file` sample used by `Software_Sync`.

## Layout

- `bin/lidar_lvx_sample`: default binary used by `scripts/software_sync_capture.py`.
- `src/`: copied sample source (`main.cpp`, `lvx_file.cpp`, `lvx_file.h`, `third_party/rapidxml`).

## Why this exists

`Software_Sync` should be independent of `Hardware_Sync`, so the LiDAR sample assets used for sync are duplicated here.

## Runtime use

`run_capture.sh` automatically uses:

```text
src/Software_Sync/lidar/bin/lidar_lvx_sample
```

You can still override the binary path via:

```bash
./run_capture.sh --frames 100 --lidar-binary /absolute/path/to/lidar_lvx_sample
```
