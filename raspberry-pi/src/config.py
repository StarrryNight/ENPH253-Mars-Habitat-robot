import os

# UART
UART_PORT = "/dev/ttyACM0"  # ESP32-S3 native USB (ARDUINO_USB_CDC_ON_BOOT) — same port the serial monitor uses
UART_BAUD = 115200

# Computer vision
CAMERA_INDEX = (
    1  # /dev/videoN — on the robot this is 0 (only camera); this dev machine needs 1
)
MODEL_PATH = os.path.expanduser(
    "../best_ncnn_model"
)  # directory containing .param and .bin files
CV_CONF = 0.04
CV_IOU = 0.7
CV_IMGSZ = 640
HFOV_DEG = 60.0  # TODO: measure actual camera horizontal FOV
TELETUBBY_LABELS = {"laa-laa", "Tinky winky", "Dipsy", "po"}
CLASS_NAMES = ["laa-laa", "Tinky winky", "Dipsy", "po"]  # index order per best_ncnn_model/metadata.yaml
