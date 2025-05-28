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
        # Define the upwind and downwind no-sail zones (relative to boat heading)
        self.upwind_zone = (315, 45)   # 45 degrees on either side of bow
        self.downwind_zone = (135, 225)  # 45 degrees on either side of stern
    
    def calculate_path(self, start_point: Tuple[float, float], 
                      end_point: Tuple[float, float],
                      wind_angle: float,  # Wind angle relative to boat
                      wind_speed: float,
                      boat_heading: float,
                      first_maneuver_is_starboard: bool = True) -> List[Tuple[float, float]]:
        """
        calculate optimal path between points considering wind relative to boat
        
        args:
            start_point: (lat, lon) of starting position
            end_point: (lat, lon) of ending position
            wind_angle: wind angle relative to boat heading in degrees (0 = head-on)
            wind_speed: wind speed
            boat_heading: current boat heading in degrees (0 = North, clockwise)
            first_maneuver_is_starboard: If True, the first tack/jibe will be on a starboard
                                         maneuver. If False, it will be port. Defaults to True.
            
        returns:
            list of waypoints including any intermediate tacking/jibing points
        """
        # Calculate desired course angle (global reference frame)
        course_angle = math.degrees(math.atan2(
            end_point[1] - start_point[1],
            end_point[0] - start_point[0]
        ))
        
        # Calculate angle between course and boat heading (relative to boat)
        course_relative_to_boat = (course_angle - boat_heading) % 360
        if course_relative_to_boat > 180:
            course_relative_to_boat -= 360
        
        # Check if the target direction is in a no-sail zone relative to the wind
        # First, normalize wind angle to 0-360 range
        wind_angle_normalized = wind_angle % 360
        
        # Determine if upwind or downwind sailing based on relative wind angle
        upwind_sailing = (wind_angle_normalized >= self.upwind_zone[0] or 
                           wind_angle_normalized <= self.upwind_zone[1])
        
        downwind_sailing = (wind_angle_normalized >= self.downwind_zone[0] and 
                             wind_angle_normalized <= self.downwind_zone[1])
        
        # Calculate the angle between our desired course and the wind
        target_to_wind_angle = (course_relative_to_boat - wind_angle_normalized) % 360
        if target_to_wind_angle > 180:
            target_to_wind_angle -= 360
        
        # For vector calculations, we need to convert to a global reference frame
        global_wind_angle = (boat_heading + wind_angle_normalized) % 360
        normalized_wind = Angle(1, global_wind_angle)
        
        # Determine if we need to tack or jibe based on wind zones and polar data
        if upwind_sailing and abs(target_to_wind_angle) < self.polar_data.upwind_vmg:
            # We're trying to sail too close to the wind - need to tack
            return self._calculate_upwind_path(
                start_point, end_point, normalized_wind, wind_speed, boat_heading, first_maneuver_is_starboard)
                
        elif downwind_sailing and abs(target_to_wind_angle) > self.polar_data.downwind_vmg:
            # We're trying to sail directly downwind - need to jibe
            return self._calculate_downwind_path(
                start_point, end_point, normalized_wind, wind_speed, boat_heading, first_maneuver_is_starboard)
                
        else:
            # Direct path is possible - no need for tacking or jibing
            return [end_point]
    
    def _calculate_upwind_path(self, start: Tuple[float, float], 
                             end: Tuple[float, float],
                             wind: Angle,
                             wind_speed: float,
                             boat_heading: float,
                             first_maneuver_is_starboard: bool) -> List[Tuple[float, float]]:
        """calculate tacking path for upwind sailing
        
        Args:
            first_maneuver_is_starboard: If True, the first tack will be starboard. Else, port.
        """
        # Create vector from start to end
        v_target = Vector(
            Angle(1, math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))), 
            math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2))
        
        # Calculate optimal tacking vectors using the polar data
        # Wind is already converted to global reference frame
        # k_starboard: starboard tack vector (wind over boat's starboard/right side)
        # j_port: port tack vector (wind over boat's port/left side)
        k_starboard = Vector(wind + Angle(1, 180 + self.polar_data.upwind_vmg), 1) 
        j_port = Vector(wind + Angle(1, 180 - self.polar_data.upwind_vmg), 1)
        
        if first_maneuver_is_starboard:
            leg1_vec = k_starboard
            leg2_vec = j_port
        else:
            leg1_vec = j_port
            leg2_vec = k_starboard
            
        # Solve system of equations to find optimal tacking point
        # v_target = scalar1 * leg1_vec + scalar2 * leg2_vec
        D_det = np.linalg.det([[leg1_vec.xcomp(), leg2_vec.xcomp()], 
                               [leg1_vec.ycomp(), leg2_vec.ycomp()]])
        
        if abs(D_det) < 1e-9: # Avoid division by zero if vectors are collinear
            # This shouldn't happen with valid VMG angles but as a fallback:
            self.get_logger().warning("Collinear tacking vectors, defaulting to direct path.")
            return [end]

        D_scalar1 = np.linalg.det([[v_target.xcomp(), leg2_vec.xcomp()], 
                                   [v_target.ycomp(), leg2_vec.ycomp()]])
        # D_scalar2 = np.linalg.det([[leg1_vec.xcomp(), v_target.xcomp()], # Not strictly needed for tack point
        #                            [leg1_vec.ycomp(), v_target.ycomp()]])
        
        scalar1 = D_scalar1 / D_det
        # scalar2 = D_scalar2 / D_det # Length along second leg
        
        # Calculate intermediate tacking point: start + leg1_vec * scalar1
        tack_point_x = start[0] + leg1_vec.xcomp() * scalar1
        tack_point_y = start[1] + leg1_vec.ycomp() * scalar1
        
        tack_point = (tack_point_x, tack_point_y)
        
        return [tack_point, end]
    
    def _calculate_downwind_path(self, start: Tuple[float, float],
                               end: Tuple[float, float], 
                               wind: Angle,
                               wind_speed: float,
                               boat_heading: float,
                               first_maneuver_is_starboard: bool) -> List[Tuple[float, float]]:
        """calculate jibing path for downwind sailing
        
        Args:
            first_maneuver_is_starboard: If True, the first jibe will be starboard. Else, port.
        """
        # Similar to upwind but using downwind vmg angles
        v_target = Vector(
            Angle(1, math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))), 
            math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2))
        
        # Wind is already converted to global reference frame
        # k_starboard: starboard jibe vector (wind over boat's starboard/right quarter)
        # j_port: port jibe vector (wind over boat's port/left quarter)
        k_starboard = Vector(wind + Angle(1, 180 + self.polar_data.downwind_vmg), 1)
        j_port = Vector(wind + Angle(1, 180 - self.polar_data.downwind_vmg), 1)
        
        if first_maneuver_is_starboard:
            leg1_vec = k_starboard
            leg2_vec = j_port
        else:
            leg1_vec = j_port
            leg2_vec = k_starboard

        # Solve system of equations for optimal jibing point
        # v_target = scalar1 * leg1_vec + scalar2 * leg2_vec
        D_det = np.linalg.det([[leg1_vec.xcomp(), leg2_vec.xcomp()], 
                               [leg1_vec.ycomp(), leg2_vec.ycomp()]])

        if abs(D_det) < 1e-9: # Avoid division by zero if vectors are collinear
            self.get_logger().warning("Collinear jibing vectors, defaulting to direct path.")
            return [end]
            
        D_scalar1 = np.linalg.det([[v_target.xcomp(), leg2_vec.xcomp()],
                                   [v_target.ycomp(), leg2_vec.ycomp()]])
        # D_scalar2 = np.linalg.det([[leg1_vec.xcomp(), v_target.xcomp()], # Not strictly needed
        #                            [leg1_vec.ycomp(), v_target.ycomp()]])
        
        scalar1 = D_scalar1 / D_det
        # scalar2 = D_scalar2 / D_det
        
        # Calculate intermediate jibing point: start + leg1_vec * scalar1
        jibe_point_x = start[0] + leg1_vec.xcomp() * scalar1
        jibe_point_y = start[1] + leg1_vec.ycomp() * scalar1
        
        jibe_point = (jibe_point_x, jibe_point_y)
        
        return [jibe_point, end]