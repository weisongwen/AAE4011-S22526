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

# Version with automatic image upload
from google.colab import files
from IPython.display import display
from PIL import Image
import matplotlib.pyplot as plt
import torch

# Install packages
!pip install -q torch torchvision torchaudio
!pip install -q ultralytics

# Upload image
print("Please upload an image file...")
uploaded = files.upload()
img_path = list(uploaded.keys())[0]

# Load model
print("Loading YOLOv5 model...")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Load image
img = Image.open(img_path)

# Display original
plt.figure(figsize=(12, 8))
plt.imshow(img)
plt.axis('off')
plt.title('Original Image')
plt.show()

# Run detection
results = model(img)
detections = results.pandas().xyxy[0]

# Count people
people_count = len(detections[detections['name'] == 'person'])

# Display results
print(f"\n{'='*50}")
print(f"🧑 Number of people detected: {people_count}")
print(f"{'='*50}\n")

# Show detailed info
if people_count > 0:
    people_detections = detections[detections['name'] == 'person']
    for idx, detection in people_detections.iterrows():
        print(f"Person {idx + 1}: Confidence = {detection['confidence']:.2%}")

# Show detection image
results.show()
