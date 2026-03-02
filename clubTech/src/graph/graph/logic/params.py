#i define the parameters useful for the code
import numpy as np


area_scene = [(0.6, 2.0), (2.4, 2.0), (2.4, 1.55), (0.6, 1.55)]
SECURITY_DISTANCE_BM = 0.220
GAME_AREA_HEIGHT = 2
GAME_AREA_WIDTH = 3 

def scene(area):
    c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)
    security_offset_up_right = SECURITY_DISTANCE_BM * np.array([c, s])
    security_offset_down_right = SECURITY_DISTANCE_BM * np.array([c, -s])
    rotation = np.identity(2)
    up_left = np.array(area[0])
    up_right = np.array(area[1])
    down_right = np.array(area[2])
    down_left = np.array(area[3])
    area_new = [tuple(up_left - rotation @ security_offset_down_right),
              tuple(up_right + rotation @ security_offset_up_right),
              tuple(down_right + rotation @ security_offset_down_right),
              tuple(down_left  - rotation @ security_offset_up_right)]

    return area_new

SCENE_AREA = scene(area_scene) # with security distances