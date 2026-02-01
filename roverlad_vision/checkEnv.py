import torchvision
import torch
print("Torchvision:", torchvision.__version__)
print("Torch:", torch.__version__)
print("CUDA:", torch.cuda.is_available())
print(torch.version.cuda)      # CUDA version PyTorch was built against
print(torch.backends.cudnn.version())  # cuDNN version
print("Devices: ", torch.cuda.device_count())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "No GPU")