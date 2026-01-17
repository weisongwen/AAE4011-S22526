"""
AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
Semester 2, 2025-2026
Week 1: Introduction of AI for Unmanned Autonomous Systems

Copyright (c) 2025-2026
Department of Aeronautical and Aviation Engineering (AAE)
The Hong Kong Polytechnic University

Lecturer: Dr. Weisong Wen
Email: welson.wen@polyu.edu.hk
Address: R820, PolyU

This code is provided for educational purposes as part of the AAE4011 course.
All rights reserved.

Note: This code is designed to run on Google Colab.
The !pip install commands and IPython syntax are specific to Colab/Jupyter environments.
"""

from IPython import get_ipython
from IPython.display import display
# %%
!pip install torch torchvision torchaudio
!pip install yolov5
# %%
import torch

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
from PIL import Image
import matplotlib.pyplot as plt

# Replace with the correct path to your image within the Colab environment
img_path = '/content/0000001_04527_d_0000008.jpg' 
img = Image.open(img_path)
plt.imshow(img)
plt.axis('off')
plt.show()
results = model(img)

results.show()
results.save('/content/pre_0000001_04527_d_0000008.jpg')
