'''
After the list has been created, all cameras are started for a live video stream and
ended after a key was hit.
'''
import json
import time
import cv2
import CAMERA
import numpy as np
import sys
import TIS
import gi
import ctypes
import csv
import signal

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
gi.require_version("GObject", "2.0")
gi.require_version("GLib", "2.0")

meta_file_path = "metadata_output.txt"
metadata_list = []

from gi.repository import Gst, GstVideo, GObject, GLib

# Load tiscamera GstMeta library
clib = ctypes.CDLL("libtcamgststatistics.so")
clib.tcam_statistics_get_structure.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
clib.tcam_statistics_get_structure.restype = ctypes.c_bool

meta_out_buffer_size = 320
meta_out_buffer = ctypes.create_string_buffer(meta_out_buffer_size)

# Flag to indicate when to stop the script
stop_script = False

def write_metadata_to_csv(filename, metadata_list):
    if not metadata_list:
        print("No metadata to write.")
        return

    headers = metadata_list[0].keys()
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=headers)
        writer.writeheader()
        for meta in metadata_list:
            writer.writerow(meta)

def get_meta(gst_buffer):
    meta = gst_buffer.get_meta("TcamStatisticsMetaApi")
    if not meta:
        return None

    ret = clib.tcam_statistics_get_structure(hash(meta), meta_out_buffer, meta_out_buffer_size)
    if ret:
        structure_string = ctypes.string_at(meta_out_buffer).decode("utf-8")
        struc = Gst.Structure.from_string(structure_string)
        return struc[0]
    return None

class CustomData:
    def __init__(self):
        self.imagecounter = 0
        self.busy = False
        self.ignore_initial_frames = True  # Add a flag to ignore initial frames

def on_new_image(camera, userdata):
    if userdata.ignore_initial_frames:
        # Ignore the first frame(s) after starting the pipeline
        userdata.ignore_initial_frames = False
        return

    sample = camera.appsink.emit("pull-sample")
    if sample:
        gst_buffer = sample.get_buffer()
        tcam_meta = get_meta(gst_buffer)
        if tcam_meta:
            current_meta = {}
            def save_structure(field_id, value, user_data):
                name = GLib.quark_to_string(field_id)
                current_meta[name] = value
                return True
            tcam_meta.foreach(save_structure, None)
            metadata_list.append(current_meta)
        else:
            print("No meta")

    if userdata.busy:
        return

    userdata.busy = True
    image = camera.get_image()
    if camera.imageprefix == "left":
        userdata.imageleft = image
        userdata.imagecounter += 1
        filenameleft = f"./image_left{userdata.imagecounter:04}.png"
        cv2.imwrite(filenameleft, userdata.imageleft)
    else:
        userdata.imageright = image
        userdata.imagecounter += 1
        filenameright = f"./image_right{userdata.imagecounter:04}.png"
        cv2.imwrite(filenameright, userdata.imageright)

    userdata.busy = False

# Signal handler to stop the script
def signal_handler(sig, frame):
    global stop_script
    stop_script = True

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Get the JSON file path from the command-line arguments
if len(sys.argv) < 2:
    print("Error: Missing JSON file path argument.")
    sys.exit(1)

json_file = sys.argv[1]

# Load the camera configurations from the JSON file
with open(json_file) as jsonFile:
    cameraconfigs = json.load(jsonFile)

# Initialize cameras
CD = CustomData()

cameras = []
for cameraconfig in cameraconfigs['cameras']:
    camera = CAMERA.CAMERA(cameraconfig['properties'], cameraconfig['imageprefix'])
    camera.open_device(cameraconfig['serial'],
                       cameraconfig['width'],
                       cameraconfig['height'],
                       cameraconfig['framerate'],
                       TIS.SinkFormats[cameraconfig['pixelformat']],
                       False)
    camera.set_image_callback(on_new_image, CD)
    cameras.append(camera)

for camera in cameras:
    camera.enableTriggerMode("Off")
    camera.start_pipeline()
    camera.applyProperties()
    camera.enableTriggerMode("On")

print("Waiting for hardware triggers...")

# Main loop to wait for hardware triggers
while not stop_script:
    time.sleep(0.1)  # Small delay to prevent high CPU usage

# Cleanup and save metadata
#print("Stopping cameras and saving metadata...")
for camera in cameras:
    camera.enableTriggerMode("Off")
    camera.stop_pipeline()

#uncomment the following line to save metadata to CSV

#write_metadata_to_csv('metadata_left.csv', metadata_list)
#print("Metadata saved. Program end.")
