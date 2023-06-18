import cv2
from skimage.metrics import structural_similarity as ssim
import numpy as np
from PIL import Image, ImageChops

################################################################
###########   USING PIXEL COMPARISON   #########################

img1= Image.open('C:/Users/jcgre/OneDrive/Ambiente de Trabalho/img/11.png')
img2= Image.open('C:/Users/jcgre/OneDrive/Ambiente de Trabalho/img/2.png')

img1 = img1.convert("RGB")
img2 = img2.convert("RGB")

#resize
width = max(img1.width, img2.width)
height = max(img1.height, img2.height)
img1 = img1.resize((width, height))
img2 = img2.resize((width, height))
# Convert the images to NumPy arrays
pixels1 = np.array(img1)
pixels2 = np.array(img2)
# Initialize a counter for the pixel differences
diff_count = np.sum(pixels1 != pixels2)
total_pixels = width * height
percentage_diff = (diff_count / (total_pixels * 3)) * 100
print("Number of different pixels: " + str(diff_count/3))
print("Percentage of equal pixels: " + str(100-percentage_diff))

################################################################
###########   USING SSIM   #####################################

# Load images
image1 = cv2.imread('C:/Users/jcgre/OneDrive/Ambiente de Trabalho/img/11.png')
image2 = cv2.imread('C:/Users/jcgre/OneDrive/Ambiente de Trabalho/img/2.png')

# Convert images to grayscale
gray_image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
gray_image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

# Normalize images to range [0, 1]
normalized_image1 = gray_image1 / 255.0
normalized_image2 = gray_image2 / 255.0

# Calculate Structural Similarity Index (SSIM)
ssim_score = ssim(normalized_image1, normalized_image2, data_range=1.0)

# Convert SSIM to percentage
diff_percentage = ssim_score * 100

print("Using SSIM: " + str(diff_percentage))

