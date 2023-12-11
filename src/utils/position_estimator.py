from math import sin, cos, radians
import numpy as np
from segmenter_ros2.msg import Pixel, Coordinates, Mask
from typing import List, Tuple

RATIO = 0.1

def estimate_position(area: Mask, img: np.ndarray, camera_pos: Tuple[float], angle: Tuple[float]) -> List[Coordinates]:

    (im_h, im_w) = img.shape

    def get_coordinates(points: List[Pixel], img: np.ndarray) -> List[Coordinates]:
        """
        transforms 2d coordinates of a pixel into 3d coordinates using a depth image
        also sets the origin to the center of the image for easier manipulation later
        """

        new = []

        for point in points:
            if img[point.y][point.x]:
                co = Coordinates()
                co.x = float(point.x - im_w)
                co.y = float(point.y - im_h)
                co.z = float(img[point.y][point.x] * RATIO)
                new.append(co)

        return new
    
    def get_relative_coordinates(points: List[Coordinates]) -> [Coordinates]:
        """
        transforms 3d coordinates of a pixel relative to an image into 
        3d coordinates of this pixel relative to the camera
        """
        
        for point in points:
            angle_x = 34.5 * abs(point.x / (im_w / 2))
            angle_y = 21 * abs(point.y / (im_h / 2))
            point.x = sin(radians(angle_x)) * point.z
            point.y = sin(radians(angle_y)) * point.z

        return points
    
    def rotate_position(points: List[Coordinates], angle: Tuple[float]) -> List[Coordinates]:
        """
        rotates the coordinates of a pixel on the x-z-plan around the origin
        """
        """rad_angle = radians(angle)
        c = cos(rad_angle)
        s = sin(rad_angle)

        for point in points:
            point.x, point.y = c * point.x - s * point.y, s * point.x + c * point.y"""
        
        cx = cos(radians(angle[0]))
        sx = sin(radians(angle[0]))
        cy = cos(radians(angle[1]))
        sy = sin(radians(angle[1]))
        cz = cos(radians(angle[2]))
        sz = sin(radians(angle[2]))

        for point in points:
            x = (cy * cz * point.x) - (cy * sz * point.y) + (sy * point.z)
            y = ((sx * sy * cz + cx * sz) * point.x) + ((cx * cz - sx * sy * sz) * point.y) - (sx * cy * point.z)
            z = ((sx * sz - cx * sy * cz) * point.x) + ((cx * sy * sz + sx * cz) * point.y) + (cx * cy * point.z)
            point.x, point.y, point.z = x, y, z

        return points
    
    def add_camera_pos(points: List[Coordinates], camera_pos: Tuple[float]) -> List[Coordinates]:
        """
        add the current position of the camera to the position of a pixel
        """
        for point in points:
            point.x += camera_pos[0]
            point.y += camera_pos[1]
            point.z += camera_pos[2]

        return points
    
    return add_camera_pos(
            rotate_position(
                get_relative_coordinates(
                    get_coordinates(area, img)
                ), 
                angle
            ),
            camera_pos)
        

