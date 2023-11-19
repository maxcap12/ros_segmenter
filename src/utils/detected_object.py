class DetectedObject:
    def __init__(self, x, y, z, w, h, angle, instances=1) -> None:
        self.x  = x
        self.y = y
        self.z = z 
        self.h = h 
        self.w = w
        self.angle = angle
        self.instance_count = instances

    def __eq__(self, __value: object) -> bool:
        """
        if the distance between the center of the objects is less than the size in the 
        corresponding axis of the biggest object, considers both objects are equal

        since the size on z axis of objects is unkown, considers that it's equel to their width
        """
        return (
            type(__value) == DetectedObject and
            abs(self.x - __value.x) < max(self.w, __value.w) and
            abs(self.y - __value.y) < max(self.h, __value.h) and
            abs(self.z - __value.z) < max(self.w, __value.w)
        )

    def __add__(self, __value: object):
        """
        merge 2 instances of the same object

        if one is contained in the other, keeps the bigger one
        else take the average of both
        """
        if type(__value) != DetectedObject:
            raise TypeError(f"Incorect type: {type(__value)}")
        
        if ( # self object is contained in __value  object
            (self.x + self.w/2) < (__value.x + __value.w/2) and
            (self.y + self.h/2) < (__value.y + __value.h/2) and
            (self.z + self.w/2) < (__value.z + __value.w/2)
        ):
            return __value
        
        if ( # __value object is contained in self  object
            (self.x + self.w/2) > (__value.x + __value.w/2) and
            (self.y + self.h/2) > (__value.y + __value.h/2) and
            (self.z + self.w/2) > (__value.z + __value.w/2)
        ):
            return self
        
        return DetectedObject(
            (self.x + __value.x) / 2,
            (self.y + __value.y) / 2,
            (self.z + __value.z) / 2,
            (self.w + __value.w) / 2,
            (self.h + __value.h) / 2,
            (self.angle + __value.angle) / 2,
            self.instance_count + 1
        )
    
    def __repr__(self) -> str:
        return f"x: {self.x} \ny: {self.y} \nz: {self.z} \nw: {self.w} \nh: {self.h}, \nangle: {self.angle}"
    