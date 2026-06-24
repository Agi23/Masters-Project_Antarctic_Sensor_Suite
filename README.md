# Antarctic Sensor Suite – Master's Research

## Overview

This repository contains the software for software-synchronized multi-sensor data capture as part of a Master's research project.
The active code lives in [`src/Software_Sync/`](src/Software_Sync/).

## Quick Start

```bash
cd src/Software_Sync
./run_capture.sh --frames 100 --lidar-format lvx
```

See [`src/Software_Sync/README.md`](src/Software_Sync/README.md) for full usage.

## Notes

- Raw sensor data (`captures/`) is excluded from version control via `.gitignore`.
- `src/Hardware_Sync/` is not tracked on this branch.
