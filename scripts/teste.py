import numpy as np
from sensor_msgs.msg import LaserScan

sensor = LaserScan()
sensor.ranges =  [1.0]*10
scan_noise = np.random.normal(1,0,10)
print(sensor.ranges)
print(scan_noise)
scan_noise = abs(scan_noise)
print(scan_noise)
sensor.ranges *= scan_noise
print(sensor.ranges)