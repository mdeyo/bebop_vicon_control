#!/usr/bin/env python

from bebop_api_client import Bebop
import time

bebop1 = Bebop('bebop1')
bebop1.takeoff()
time.sleep(3)

path = [(-2, 1.4), (-1.9, 0.3), (-1.39, 0.6),
        (-1.25, 1.43), (-0.72, 1.35), (-.58, 0.65)]
for point in path:
    print point
    bebop1.fly_to(point[0], point[1], 0.9)
    time.sleep(4)

time.sleep(3)
bebop1.land()
