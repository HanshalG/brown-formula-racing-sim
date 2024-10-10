import cv2
import numpy as np
from matplotlib import pyplot as plt

# Load the image
image_path = "input_image2.PNG"
image = cv2.imread(image_path)

# Convert image to RGB (OpenCV loads images as BGR)
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Convert image to HSV (hue, saturation, value) for color segmentation
hsv_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)

# Define the lower and upper bounds for red color in HSV
lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])
lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])

# Create masks for red color (due to the circular nature of hue in HSV)
mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
red_mask = mask1

# Apply the mask to the image to isolate the red track
red_track = cv2.bitwise_and(image_rgb, image_rgb, mask=red_mask)

# Optionally convert back to BGR if you want to save using OpenCV
red_track_bgr = cv2.cvtColor(red_track, cv2.COLOR_RGB2BGR)

# Display the result using matplotlib (or you can use OpenCV's imshow if running locally)
plt.figure(figsize=(10, 6))
plt.imshow(red_track)
plt.axis('off')
plt.show()

# Save the output if needed
output_path = "cleaned_red_track2.png"
cv2.imwrite(output_path, red_track_bgr)
print(f"Cleaned image saved at {output_path}")
