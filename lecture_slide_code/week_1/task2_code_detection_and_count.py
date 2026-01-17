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
from PIL import Image
import matplotlib.pyplot as plt
import torch

# Install required packages
print("Installing required packages...")
!pip install -q torch torchvision torchaudio
!pip install -q ultralytics

# Load YOLOv5 model
print("Loading YOLOv5 model...")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Upload image (uncomment if you want to upload from local)
# from google.colab import files
# uploaded = files.upload()
# img_path = list(uploaded.keys())[0]

# Or use a sample image path
img_path = '/content/test.jpg'  # Make sure to upload your image first

# Load and display the original image
print(f"Loading image from: {img_path}")
img = Image.open(img_path)

plt.figure(figsize=(12, 8))
plt.imshow(img)
plt.axis('off')
plt.title('Original Image')
plt.show()

# Run detection
print("Running YOLOv5 detection...")
results = model(img)

# Get detection results
detections = results.pandas().xyxy[0]  # Get pandas DataFrame with detections

# Count people detected (class name 'person' in COCO dataset)
people_count = len(detections[detections['name'] == 'person'])

print("\n" + "="*50)
print(f"Number of people detected: {people_count}")
print("="*50)

# Display detailed detection information
if people_count > 0:
    print("\nDetailed person detections:")
    people_detections = detections[detections['name'] == 'person']
    for idx, detection in people_detections.iterrows():
        print(f"Person {idx + 1}: Confidence = {detection['confidence']:.2f}")
else:
    print("\nNo people detected in the image.")

# Display all detections summary
print(f"\nTotal objects detected: {len(detections)}")
print("\nAll detected objects:")
print(detections[['name', 'confidence']].to_string(index=False))

# Show results with bounding boxes
print("\nDisplaying detection results...")
results.show()

# Save the result
output_path = '/content/detection_result.jpg'
results.save(output_path)
print(f"\nResults saved to: {output_path}")

# Display the result image
result_img = Image.open(output_path + '/image0.jpg')  # YOLOv5 saves in a folder
plt.figure(figsize=(12, 8))
plt.imshow(result_img)
plt.axis('off')
plt.title(f'Detection Results - {people_count} People Detected')
plt.show()
