#!/usr/bin/python3
from matplotlib import cm
import numpy as np
import cv2

# camera calibration
fx = 517
fy = 517
cx = 320
cy = 240
# image size
im_width = 640
im_height = 480
# frustum clipping (in mm)
clip_near = 200
clip_far = 3000

stripe_height = 32
vertical_margin_size = (im_height - stripe_height) // 2
stripes_depth = np.zeros((im_height, im_width), dtype=np.uint16)

stripe_count = (im_width - 20) // 10

distance_between_stripes = 40
first_stripe_at_depth = clip_near + distance_between_stripes
for i_stripe in range(stripe_count):
    stripes_depth[vertical_margin_size:-vertical_margin_size, (10 + 10 * i_stripe): (10 + 10 * (i_stripe + 1))] = \
        first_stripe_at_depth + distance_between_stripes * i_stripe

cv2.imwrite("stripes_depth.png", stripes_depth)

progression = (stripes_depth.astype(np.float64) - clip_near)
progression[progression < 0] = 0
progression = progression / progression.max()
stripe_index = (progression * 62).astype(np.int64)

# For grayscale stripes:
# stripes_color = np.zeros((im_height,im_width,3),dtype=np.uint8)

# for i_channel in range(3):
# 	stripes_color[:,:,i_channel] = progression.astype(np.uint8)

stripe_colors = (np.fliplr(
    np.array([cm.cividis(value)[0:3] for value in np.arange(0, 1.0, 1.0 / stripe_count)])) * 255).astype(np.uint8)

# For stripes with consecutively close colors:
# stripes_color = (cm.cividis(progression)[:,:,0:3] * 255).astype(np.uint8)

# For stripes with highly-varying colors:
new_stripe_colors = np.zeros((stripe_colors.shape[0] + 1, stripe_colors.shape[1]), dtype=np.uint8)

for i_color in range(stripe_count):
    new_stripe_colors[i_color + 1] = stripe_colors[i_color // 2] if i_color % 2 == 0 else stripe_colors[
        stripe_count // 2 + i_color // 2]

stripes_color = new_stripe_colors[stripe_index]

cv2.imwrite("stripes_color.png", stripes_color)


# u is the horizontal image coordinate
# _c denotes "in camera space"
# coordinates are expressed in mm


def z_c(u):
    return first_stripe_at_depth + distance_between_stripes * ((u - 10) // 10)


def x_c(u):
    return (z_c(u) * (u - cx)) / fx
