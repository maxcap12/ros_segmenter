from utils.depth_estimator import estimate_depth
import time
from math import sin, radians

def estimate_position(area, img):
    # estimates the position of an object relative to the camera
    t = time.time()
    distance = estimate_depth(area, img)
    print("\n", distance)
    x1, x2, y1, y2 = _get_boundings(area)
    
    center = ((x1 + x2)/2, (y1 + y2)/2)
    (im_h, im_w) = img.shape

    # we take (0, 0) as the center of the image
    center = (center[0] - (im_w / 2), center[1] - (im_h / 2))

    print(center)

    # position of the center of the object relative to the camera
    angle_x = 34.5 * abs(center[0] / (im_w / 2))
    angle_y = 21 * abs(center[1] / (im_h / 2))
    center_x = sin(radians(angle_x)) * distance
    center_y = sin(radians(angle_y)) * distance

    # width and hieght of the object
    obj_w = (center_x * (x2 - x1)) / abs(center[0]) 
    obj_h = (center_y * (y2 - y1)) / abs(center[1])

    print(f"estimation {time.time()-t}")
    return center_x, center_y, distance, obj_w, obj_h

def _get_boundings(area):
    max_h = area[0].y
    min_h = area[0].y
    max_w = area[0].x
    min_w = area[0].x

    for p in area:

        if p.x < min_w:
            min_w = p.x
        elif p.x > max_w:
            max_w = p.x
        if p.y < min_h:
            min_h = p.y
        elif p.y > max_h:
            max_h = p.y

    return min_w, max_w, min_h, max_h
        

        