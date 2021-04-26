import cv2
import time
from time import sleep
import numpy as np
from math import sin,cos
import pygame

import os
from tqdm import tqdm
import random
import open3d as o3d
from numba import jit, njit
import sys
import pandas as pd

from math import sin, cos, tan
import matplotlib.pyplot as plt


reader = o3d.io.AzureKinectMKVReader()
reader.open("test.mkv")
print(reader.get_metadata().stream_length_usec)

save_to_images = False
file_to_save = "mkv_images"
if save_to_images:
    if not os.path.exists(file_to_save):
        os.mkdir(file_to_save)


counter = 0
while True:
    try:
        rgbd = reader.next_frame()
        if reader.is_eof():
            break
        color = np.asarray(rgbd.color).astype(np.uint8)
        depth = np.asarray(rgbd.depth).astype(np.float32) / 1000.0


        cv2.imshow("color", cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
        cv2.imshow("depth", depth)
        cv2.waitKey(1)

        if save_to_images:
            plt.imsave(file_to_save + 'depth_png_' + str(counter).zfill(5) + '.png', depth)
            np.save(file_to_save + 'depth_' + str(counter).zfill(5) + '.npy', depth)
            plt.imsave(file_to_save + 'color_' + str(counter).zfill(5) + '.png', color)
        counter += 1

    except Exception as e:
        print(e)
        break

print(counter, " many frames where processed")
