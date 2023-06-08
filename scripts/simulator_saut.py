import pygame
import numpy as np
from math import floor
from PIL import Image

image = Image.open('image2.png')
# Convert the image to grayscale
image = image.convert('L')
# Convert the image to a 2D NumPy array
array = np.array(image)
# Print the shape of the array
print(array.shape)
width = array.shape[1]
height = array.shape[0]
print(array)
map = array
# map = np.flip(array, 0)


# Define the dimensions of the screen
SCREEN_WIDTH = 700
SCREEN_HEIGHT = 700

# Define the dimensions of the array
ARRAY_COLS = array.shape[1]
ARRAY_ROWS = array.shape[0]
# Calculate the size of each cell in the array
CELL_WIDTH = SCREEN_WIDTH // ARRAY_COLS
CELL_HEIGHT = SCREEN_HEIGHT // ARRAY_ROWS

GRID_SIZE = 1 #metros
PIXEL_SIZE = CELL_HEIGHT / GRID_SIZE #pixel/metro

#posicoes em metros 
pos = np.array([26.1, 25.1])
dir = np.array([0.0, 1.0])
vl = 1
va = 0
min_omega = np.pi/12
omega = 0
rot = np.zeros((2,2))
rate = 10
delta = 1/rate
delay = int(1/rate *1000)


#SENSOR DATA
ranges = [0.0]*360
angle_increment = 2*np.pi/360
rot_lidar = np.zeros((2,2))
rot_lidar[0,0] = np.cos(angle_increment)
rot_lidar[0,1] = -np.sin(angle_increment)
rot_lidar[1,0] = np.sin(angle_increment)
rot_lidar[1,1] = np.cos(angle_increment)

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()

# Game loop
running = True
while running:
    pygame.time.delay(delay)
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_a:
            if va < 3:
                va += 1            
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_d:
            if va > -3:
                va -= 1
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_w:
            if vl < 3:
                vl += 1
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
            if vl > 0:
                vl -= 1
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            va = 0
            vl = 0

    # Clear the screen
    screen.fill((255, 255, 255))

    # Draw the array
    for row in range(ARRAY_ROWS):
        for col in range(ARRAY_COLS):
            # Calculate the position of the current cell
            x = col * CELL_WIDTH
            y = row * CELL_HEIGHT
            # Draw a rectangle for the cell
            pygame.draw.rect(screen, (array[row,col],array[row,col],array[row,col]), (x, y, CELL_WIDTH, CELL_HEIGHT))


    #draw position
    pos += delta*vl*dir
    omega = delta*va*min_omega
    rot[0,0] = np.cos(omega)
    rot[0,1] = -np.sin(omega)
    rot[1,0] = np.sin(omega)
    rot[1,1] = np.cos(omega)
    dir = rot.dot(dir)
    # pygame.draw.circle(screen, (255,0,0), pos*PIXEL_SIZE, 20)
    # pygame.draw.line(screen, (0,0,255), pos*PIXEL_SIZE, (pos*PIXEL_SIZE + dir*20), 2)

    #Sensor Model
    ray_dir = dir
    for i in range(360):

        # if i == 179:
        #     print()
        # norm_dist = scan_dist/self.map.grid_size
        if i > 0:
            ray_dir = rot_lidar.dot(ray_dir)
        
        xa = pos[0]/GRID_SIZE
        ya = pos[1]/GRID_SIZE
        x_versor = ray_dir[0]
        y_versor = ray_dir[1]
        dist = 0

        if x_versor > 0: xi = 1
        else : xi = 0
        if y_versor > 0: yi = 1
        else : yi = 0

        tracing = True
        while tracing:
            x_prev = xa
            y_prev = ya
            dist_prev = dist
            #intercepcao linha vertical
            xiv = floor(xa)
            xiv += xi
            if x_versor == 0:
                k_v = 10
            else:
                k_v = (xiv-xa)/x_versor
            yiv = k_v*y_versor + ya
            #intercepcao linha horizontal
            yih = floor(ya) + yi
            if y_versor == 0:
                k_h  = 10
            else:
                k_h = (yih-ya)/y_versor
            xih = k_h*x_versor + xa
            #decidir qual a celula seguinte
            if k_h < k_v:
                #Intersecao Horizontal
                xa = xih
                ya = yih
                dist = dist + k_h
                if xi == -1: xi = 0
                if yi == 0: yi = -1
            else:
                #Intersecao Vertical
                xa = xiv
                ya = yiv
                dist = dist + k_v
                if yi == -1: yi = 0
                if xi == 0: xi = -1

            x_med = (xa+x_prev)/2
            y_med = (ya+y_prev)/2

            if map[int(y_med),int(x_med)] == 0:
                ranges[i] = dist_prev*GRID_SIZE
                tracing = False
                pygame.draw.line(screen, (0,255,0), pos*PIXEL_SIZE, (x_prev*GRID_SIZE*PIXEL_SIZE,y_prev*GRID_SIZE*PIXEL_SIZE), 1)

    pygame.draw.circle(screen, (255,0,0), pos*PIXEL_SIZE, 15)
    pygame.draw.line(screen, (0,0,255), pos*PIXEL_SIZE, (pos*PIXEL_SIZE + dir*15), 2)
    # Update the display
    display_surface = pygame.display.get_surface()
    display_surface.blit(pygame.transform.flip(display_surface, False, True), dest=(0, 0))
    pygame.display.flip()

# Quit the program
pygame.quit()