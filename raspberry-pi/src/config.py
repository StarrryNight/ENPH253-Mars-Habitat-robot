import os

# UART
UART_PORT = "/dev/ttyUSB0"
UART_BAUD = 115200

# Computer vision
CAMERA_INDEX = (
    1  # /dev/videoN — on the robot this is 0 (only camera); this dev machine needs 1
)
MODEL_PATH = os.path.expanduser(
    "~/best_ncnn_model"
)  # directory containing .param and .bin files
CV_CONF = 0.04
CV_IOU = 0.7
CV_IMGSZ = 640
HFOV_DEG = 60.0  # TODO: measure actual camera horizontal FOV
TELETUBBY_LABELS = {"Full Teletubby", "Half Teletubby"}
