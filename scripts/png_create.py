import numpy as np
from PIL import Image

# Create a random NumPy array of size 100x100
array = np.random.randint(127, 128, size=(50, 50), dtype=np.uint8)

# Create a PIL Image object from the NumPy array
image = Image.fromarray(array)

# Save the image as a .png file
image.save("image.png")