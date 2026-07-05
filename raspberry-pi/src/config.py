# UART
UART_PORT = "/dev/ttyUSB0"
UART_BAUD = 115200

# Computer vision
MODEL_PATH = "model_ncnn_model"  # directory containing .param and .bin files
CV_CONF    = 0.25
CV_IOU     = 0.7
CV_IMGSZ   = 640
HFOV_DEG   = 60.0  # TODO: measure actual camera horizontal FOV

# State timeouts
METAL_DETECT_TIMEOUT_S = 15.0
HABITAT_TIMEOUT_S      = 45.0
