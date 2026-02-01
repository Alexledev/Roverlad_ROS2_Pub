import sounddevice as sd
from load_config import ConfigLoader

# LOG_FILE = "/home/jetsonalex/Roverlad_ROS2/dashboard/log/loggus.txt"

# def log(msg: str):
#     with open(LOG_FILE, "a") as f:
#         f.write(msg + "\n")

def choose_pipewire_device():
    devices = sd.query_devices()
    # log(f"Available Devices: {devices}")

    # Look for a device explicitly named "pipewire" with input channels
    for idx, dev in enumerate(devices):
        if dev["max_input_channels"] > 0 and "pipewire" in dev["name"].lower():
            print("Using PipeWire device:", dev["name"])
            # log(f"Using PipeWire device: {dev['name']}")
            return idx
        
    cfgl = ConfigLoader("devices", "mic")
    id = cfgl.get("index")
    # log(f"Pipewire device not found, using device id {id}")
    return id