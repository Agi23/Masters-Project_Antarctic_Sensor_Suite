# Stereo_V2 Hardware Trigger Capture

This folder contains a stereo capture workflow for two The Imaging Source cameras triggered by an external Arduino pulse. The Arduino can now be driven directly by LiDAR frame-complete messages over USB serial.

## Files

- `stereo_hw_trigger_capture.py`: main stereo capture script.
- `cameras_stereo_hw.json`: camera + trigger config template (already filled with your two serials).
- `inspect_trigger_properties.py`: prints trigger-related properties for your camera model.
- `camera_recovery.py`: quickly disables trigger mode and restores safe exposure settings.
- `CAMERA.py`, `TIS.py`: camera wrapper modules.
- `arduino_trigger/arduino_trigger.ino`: Arduino sketch that emits one trigger pulse for each `F\n` line received over serial.

## 1) Check trigger enum values on your exact camera model

From this folder:

```bash
python3 inspect_trigger_properties.py --serial 44814142
python3 inspect_trigger_properties.py --serial 44814140
```

Look at values for:

- `TriggerSelector`
- `TriggerSource` (if exposed on your camera model)
- `TriggerActivation`
- `TriggerMode`

If needed, edit `cameras_stereo_hw.json`.

## 2) Wire Arduino trigger output

- Arduino trigger output pin: `A5` (from the `.ino` sketch)
- Arduino GND must be connected to camera trigger input ground.
- A5 goes to both cameras' trigger input line (the line selected by `TriggerSource`, often `Line0` or `Line1`).
- USB from host to Arduino is required so LiDAR software can send frame events.

## 3) Run LiDAR capture with Arduino serial enabled

Example:

```bash
./lidar_lvx_sample --time 60 --format lvx --arduino /dev/ttyACM0 --baud 115200
```

Each saved LiDAR frame sends `F\n` to Arduino, and Arduino generates one trigger pulse for the stereo cameras.

## 4) Run stereo capture

```bash
python3 stereo_hw_trigger_capture.py --config cameras_stereo_hw.json --output-dir captures
```

Stop with `Ctrl+C`.

Output:

- images in `captures/capture_YYYYMMDD_HHMMSS/`
- metadata in `capture_metadata.csv` in the same folder (unless `--no-metadata` is used)

## Notes

- If triggers do not fire, first verify `TriggerSource` enum (`Line0` vs `Line1`) with `inspect_trigger_properties.py`.
- If your camera does not expose `TriggerSource`, keep `"source_property": null` in config (fixed hardware input line).
- On your current cameras (serials `44814142` and `44814140`), `TriggerSource` includes `Line1` and not `Line0`.
- Pulse width is set in `arduino_trigger/arduino_trigger.ino`; pulse rate follows LiDAR frame-save timing.
- Camera exposure time must fit within your trigger period.
- If the image turns white or blank after experiments, run:

```bash
python3 camera_recovery.py --serial 44814142 --serial 44814140
```

This sets `TriggerMode=Off`, `ExposureAuto=Continuous`, and `GainAuto=Continuous` for both cameras.
