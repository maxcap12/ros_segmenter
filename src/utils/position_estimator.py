from math import sin, cos, radians
import numpy as np
from segmenter_ros2.msg import Pixel, Coordinates, Mask

darkest = 0
lightest = 10000
max_dist = 500 # TBD
min_dist = 28
ratio = (max_dist - min_dist) / (lightest - darkest)

def estimate_position(area: Mask, img: np.ndarray, camera_pos: (float)) -> [Coordinates]:

    (im_h, im_w) = img.shape

    def get_coordinates(pixel: Pixel, img: np.ndarray) -> Coordinates:
        """
        transforms 2d coordinates of a pixel into 3d coordinates using a depth image
        also sets the origin to the center of the image for easier manipulation later
        """
        co = Coordinates()
        co.x = float(pixel.x - im_w)
        co.y = float(pixel.y - im_h)
        co.z = float(img[pixel.y][pixel.x] * ratio)
        return co
    
    def get_relative_coordinates(co: Coordinates) -> Coordinates:
        """
        transforms 3d coordinates of a pixel relative to an image into 
        3d coordinates of this pixel relative to the camera
        """
        angle_x = 34.5 * abs(co.x / (im_w / 2))
        angle_y = 21 * abs(co.y / (im_h / 2))
        co.x = sin(radians(angle_x)) * co.z
        co.y = sin(radians(angle_y)) * co.z
        return co
    
    def rotate_position(co: Coordinates, angle: int) -> [Coordinates]:
        """
        rotates the coordinates of a pixel on the x-z-plan around the origin
        """
        rad_angle = radians(angle)
        c = cos(rad_angle)
        s = sin(rad_angle)
        co.x, co.y = c * co.x - s * co.y, s * co.x + c * co.y
        return co
    
    def add_camera_pos(co, camera_pos):
        """
        add the current position of the camera to the position of a pixel
        """
        co.x += camera_pos[0]
        co.y += camera_pos[1]
        co.z += camera_pos[2]
        return co
    
    return [
        add_camera_pos(
            rotate_position(
                get_relative_coordinates(
                    get_coordinates(pixel, img)
                ), 
                camera_pos[3]
            ),
            camera_pos)
        for pixel in area]
        

