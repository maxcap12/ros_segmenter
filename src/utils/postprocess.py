from typing import List
from segmenter_ros2.msg import Coordinates
from math import sqrt

def dist(point1: Coordinates, point2: Coordinates) -> float:
    """calculates the distance between 2 points"""
    return sqrt(pow(point2.x - point1.x, 2) + 
                pow(point2.y - point1.y, 2) +
                pow(point2.z - point1.z, 2))

def average(points: List[Coordinates]) -> Coordinates:
    new = Coordinates()
    new.x, new.y, new.z = 0, 0, 0

    for point in points:
        new.x += point.x
        new.y += point.y
        new.z += point.z

    new.x /= len(points)
    new.y /= len(points)
    new.z /= len(points)

    return new

def postprocess1(points: List[Coordinates], min_dist: float, ignored_dist: float=None) -> List[Coordinates]:
    """
    """
    index = 0
    nearby_points = []

    ignored_dist = ignored_dist if ignored_dist is not None else min_dist * 5

    while index + len(nearby_points) + 1 < len(points):
        if dist(points[index], points[index + len(nearby_points) + 1]) < min_dist:
            nearby_points.append(points.pop(index + len(nearby_points) + 1))
        else:
            nearby_points.append(points[index])
            points[index] = average(nearby_points)
            nearby_points = []
            index += 1

    if nearby_points:
        nearby_points.append(points[index])
        points[index] = average(nearby_points)

    return points

def postprocess2(points: List[Coordinates], min_dist: float, ignored_dist: float=None) -> List[Coordinates]:
    index = 0
    nearby_points = []
    ignored_dist = ignored_dist if ignored_dist is not None else min_dist * 5

    while index < len(points):
        for i in range(len(points)-1, index, -1):
            if dist(points[index], points[i]) < min_dist:
                nearby_points.append(points.pop(i))
        
        nearby_points.append(points[index])
        points[index] = average(nearby_points)
        nearby_points = []
        index += 1

    return points


def postprocess3(points: List[Coordinates], precision: float) -> List[Coordinates]:
# take average pos
    pass
