import math
import numpy as np
import os
from dataclasses import dataclass
from typing import List, Tuple, Optional
import csv
from .geometry_utils import Angle, Vector

class PolarData:
    """manages boat performance data from polar diagram
    
    I'm not a fan of the whole system of relying on relatively outdated polar data calculated using the sim coefficents
    But I don't really have any better ideas with current standings. After we do an on water test to see how accurate the
    simulated polars are, we can evaluate the priority of changing the polar system. 

    If the polars are inaccurate, this will probably create the need for a python script that can take the
    boat information and generate the polars much easier. This should be done regardless, but when it will be done 
    is TBD
    """
    def __init__(self):
        self.tws_values = []  # true wind speeds
        self.twa_values = []  # true wind angles
        self.speed_matrix = []  # boat speeds for each twa/tws combination
        self.upwind_vmg = None  # maxvmg upwind angle
        self.downwind_vmg = None  # maxvmg downwind angle
        
        # load polar data from file
        package_dir = os.path.dirname(os.path.dirname(__file__))
        pol_path = os.path.join(package_dir, 'data', 'test.pol')
        self._load_from_file(pol_path)
        
    def _load_from_file(self, filepath: str):
        """load and parse polar data from file"""
        with open(filepath, 'r') as f:
            lines = f.readlines()
            
        # parse header for true wind speeds
        header = lines[0].strip().split(';')
        self.tws_values = [float(x) for x in header[1:]]
        
        # parse boat speeds and true wind angles
        for line in lines[1:-1]:  # exclude last line which contains MAXVMG
            values = line.strip().split(';')
            self.twa_values.append(float(values[0]))
            self.speed_matrix.append([float(x) for x in values[1:]])
            
        # parse max vmg angles
        maxvmg = lines[-1].strip().split(';')[1:]
        self.upwind_vmg = float(maxvmg[0])
        self.downwind_vmg = float(maxvmg[1])
    
    def get_boat_speed(self, twa: float, tws: float) -> float:
        """interpolate boat speed for given true wind angle and speed"""
        # find nearest indices
        twa_idx = min(range(len(self.twa_values)), key=lambda i: abs(self.twa_values[i] - abs(twa)))
        tws_idx = min(range(len(self.tws_values)), key=lambda i: abs(self.tws_values[i] - tws))
        
        return self.speed_matrix[twa_idx][tws_idx]

class Leg:
    """calculates optimal path between waypoints considering wind conditions"""
    def __init__(self):
        self.polar_data = PolarData()
    
    def calculate_path(self, start_point: Tuple[float, float], 
                      end_point: Tuple[float, float],
                      wind_angle: float,
                      wind_speed: float) -> List[Tuple[float, float]]:
        """
        calculate optimal path between points considering wind
        
        args:
            start_point: (lat, lon) of starting position
            end_point: (lat, lon) of ending position
            wind_angle: true wind angle in degrees
            wind_speed: true wind speed
            
        returns:
            list of waypoints including any intermediate tacking/jibing points
        """
        # calculate course angle relative to wind
        course_angle = math.degrees(math.atan2(
            end_point[1] - start_point[1],
            end_point[0] - start_point[0]
        ))
        relative_angle = (course_angle - wind_angle) % 360
        if relative_angle > 180:
            relative_angle -= 360
            
        # normalize wind angle for calculations
        normalized_wind = Angle(1, wind_angle)
        
        # check if upwind or downwind scenario
        if abs(relative_angle) < self.polar_data.upwind_vmg:
            # upwind case - need to tack
            return self._calculate_upwind_path(
                start_point, end_point, normalized_wind, wind_speed)
            
        elif abs(relative_angle) > self.polar_data.downwind_vmg:
            # downwind case - need to jibe
            return self._calculate_downwind_path(
                start_point, end_point, normalized_wind, wind_speed)
            
        else:
            # direct path possible
            return [end_point]
    
    def _calculate_upwind_path(self, start: Tuple[float, float], 
                             end: Tuple[float, float],
                             wind: Angle,
                             wind_speed: float) -> List[Tuple[float, float]]:
        """calculate tacking path for upwind sailing"""
        # convert to vector math
        v = Vector(
            Angle(1, math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))), math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2))
        
        # calculate optimal tacking angles using polars
        k = Vector(wind + Angle(1, 180 + self.polar_data.upwind_vmg), 1)
        j = Vector(wind + Angle(1, 180 - self.polar_data.upwind_vmg), 1)
        
        # solve system of equations for tacking distances
        D = np.linalg.det([[k.xcomp(), j.xcomp()], [k.ycomp(), j.ycomp()]])
        Dk = np.linalg.det([[v.xcomp(), j.xcomp()], [v.ycomp(), j.ycomp()]])
        Dj = np.linalg.det([[k.xcomp(), v.xcomp()], [k.ycomp(), v.ycomp()]])
        
        a = Dk/D  # distance along first tack
        b = Dj/D  # distance along second tack
        
        # calculate intermediate tacking point
        k.norm *= a
        j.norm *= b
        tack_point = (start[0] + k.xcomp(), start[1] + k.ycomp())
        
        return [tack_point, end]
    
    def _calculate_downwind_path(self, start: Tuple[float, float],
                               end: Tuple[float, float], 
                               wind: Angle,
                               wind_speed: float) -> List[Tuple[float, float]]:
        """calculate jibing path for downwind sailing"""
        # similar to upwind but using downwind vmg angles
        v = Vector(
            Angle(1, math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))), math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2))
        
        k = Vector(wind + Angle(1, 180 + self.polar_data.downwind_vmg), 1)
        j = Vector(wind + Angle(1, 180 - self.polar_data.downwind_vmg), 1)
        
        D = np.linalg.det([[k.xcomp(), j.xcomp()], [k.ycomp(), j.ycomp()]])
        Dk = np.linalg.det([[v.xcomp(), j.xcomp()],[v.ycomp(), j.ycomp()]])
        Dj = np.linalg.det([[k.xcomp(), v.xcomp()], [k.ycomp(), v.ycomp()]])
        
        a = Dk/D
        b = Dj/D
        
        k.norm *= a
        j.norm *= b
        jibe_point = (start[0] + k.xcomp(), start[1] + k.ycomp())
        
        return [jibe_point, end]