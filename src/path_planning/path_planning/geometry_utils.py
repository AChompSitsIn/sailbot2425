import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Vector:
    """represents a vector with angle and magnitude"""
    angle: 'Angle'
    norm: float  # magnitude

    def xcomp(self) -> float:
        """get x component of vector"""
        return self.norm * math.cos(math.radians(self.angle.calc()))
    
    def ycomp(self) -> float:
        """get y component of vector"""
        return self.norm * math.sin(math.radians(self.angle.calc()))
    
    def __mul__(self, other: 'Vector') -> float:
        """dot product between two vectors"""
        return self.norm * other.norm * math.cos(math.radians(self.angle.calc() - other.angle.calc()))

class Angle:
    """represents an angle in degrees"""
    def __init__(self, sign: int, angle: float):
        self.sign = sign
        self.angle = angle
    
    def calc(self) -> float:
        """calculate actual angle value"""
        return self.sign * self.angle
    
    def __add__(self, other: 'Angle') -> 'Angle':
        """add two angles"""
        return Angle(1, self.calc() + other.calc())
    
    @staticmethod
    def norm(angle: 'Angle') -> 'Angle':
        """normalize angle to be between -180 and 180"""
        calc = angle.calc()
        while calc > 180:
            calc -= 360
        while calc < -180:
            calc += 360
        return Angle(1, calc)

def print_angle(angle: float) -> float:
    """normalize angle print format"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

class BoatPolars:
    """represents boat performance characteristics at different wind angles"""
    def __init__(self):
        # format: (angle, speed) pairs where:
        # - first element is minimum upwind angle
        # - last element is maximum downwind angle
        self.polars: List[Tuple[float, float]] = [
            # idk i could have the polars here and be lazy or i could parse a file not sure will decide l8tr
        ]
    
    @property
    def min_upwind_angle(self) -> float:
        """get minimum upwind sailing angle"""
        return self.polars[0][0]
    
    @property
    def max_downwind_angle(self) -> float:
        """get maximum downwind sailing angle"""
        return self.polars[-1][0]

    def get_boat_speed(self, wind_angle: float) -> float:
        """get boat speed at given wind angle"""
        # interpolate between polar points
        for i in range(len(self.polars) - 1):
            if self.polars[i][0] <= abs(wind_angle) <= self.polars[i + 1][0]:
                ratio = (abs(wind_angle) - self.polars[i][0]) / (self.polars[i + 1][0] - self.polars[i][0])
                return self.polars[i][1] + ratio * (self.polars[i + 1][1] - self.polars[i][1])
        return 0.0  # angle outside polar range