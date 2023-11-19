import numpy as np

darkest = 0
lightest = 10000
max_dist = 500 # TBD
min_dist = 28
ratio = (max_dist - min_dist) / (lightest - darkest)

def estimate_depth(area: list, img: np.ndarray) -> float:
    total = 0.
    count = 0
    for p in area:
        total += img[p.y][p.x]
        count += 1

    # estimates the average distance of a list of pixels relative to the camera
    return (total/count) * ratio    