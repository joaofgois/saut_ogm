import cv2
from skimage.metrics import structural_similarity as ssim
import numpy as np
from PIL import Image, ImageChops
import matplotlib.pyplot as plt

################################################################
###########   USING PIXEL COMPARISON   #########################

#img1= Image.open('C:/Users/jc/Documents/GitHub/saut_ogm/image-comparison/11.png')
#img2= Image.open('C:/Users/jc/Documents/GitHub/saut_ogm/image-comparison/2.png')

img1= Image.open('C:/Users/jc/Documents/GitHub/saut_ogm/image-comparison/mapa5.png')
img2= Image.open('C:/Users/jc/Documents/GitHub/saut_ogm/image-comparison/mapa5n.png')

#Convert to RGB
img1 = img1.convert("RGB")
img2 = img2.convert("RGB")

#resize images
width = max(img1.width, img2.width)
height = max(img1.height, img2.height)
img1 = img1.resize((width, height))
img2 = img2.resize((width, height))

# Convert the images to NumPy arrays
pixels1 = np.array(img1)
pixels2 = np.array(img2)

# Find non-white pixels in img1 and convert them to black
non_white_pixels_1 = np.any(pixels1 != [255, 255, 255], axis=2)
pixels1[non_white_pixels_1] = [0, 0, 0]
non_white_pixels_2 = np.any(pixels2 != [255, 255, 255], axis=2)
pixels2[non_white_pixels_2] = [0, 0, 0]

# Initialize a counter for the pixel differences
diff_count = np.sum(pixels1 != pixels2)
total_pixels = width * height
percentage_diff = (diff_count / (total_pixels * 3)) * 100

#Print results
print("Number of different pixels: " + str(diff_count/3))
print("Percentage of equal pixels: " + str(100-percentage_diff))

# Create new images with the modified pixels and save them
modified_img1 = Image.fromarray(pixels1)
modified_img2 = Image.fromarray(pixels2)
modified_img1.save("img1_with_black_pixels.png")
modified_img2.save("img2_with_black_pixels.png")

# Open the saved images using the default image viewer
import os
os.system("img1_with_black_pixels.png")
os.system("img2_with_black_pixels.png")