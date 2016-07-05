#!/usr/bin/env python
"""
Test the warping module generated via SWIG.
"""

import warping
import timeit

warping.init(1)

# Initialize and fill 1d images
ss = [255, 0, 0, 0, 0, 0]
cv = [255, 255, 0, 0, 0, 255]

print("set_snapshot.")
warping.set_snapshot(0, ss)
print("compute_home_vector.")
warping.compute_home_vector(0, cv)

hx = warping.get_home_x()
hy = warping.get_home_y()
print hx, hy
