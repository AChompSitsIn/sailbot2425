import math
from typing import List, Tuple, Optional, Set
from path_planning.path_planning.waypoint import Waypoint

def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    calculate the distance in meters between two gps points using the haversine formula
    
    args:
        lat1, lon1: coordinates of first point in decimal degrees
        lat2, lon2: coordinates of second point in decimal degrees
        
    returns:
        distance in meters
    """
    # earth radius in meters
    r = 6371000
    
    # convert to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    # haversine formula
    a = math.sin(delta_phi/2) * math.sin(delta_phi/2) + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda/2) * math.sin(delta_lambda/2)
        
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    # distance in meters
    distance = r * c
    
    return distance

class BuoyDetector:
    def __init__(self, threshold_distance: float = 5.0):
        """
        initialize buoy detector
        
        args:
            threshold_distance: distance in meters within which a buoy is considered reached
        """
        self.threshold_distance = threshold_distance
        self.detected_buoys: Set[Waypoint] = set()  # track buoys that have been reached
        self.current_buoy: Optional[Waypoint] = None  # most recently passed buoy
    
    def check_buoy_proximity(self, current_position: Tuple[float, float], 
                             buoys: List[Waypoint]) -> List[Waypoint]:
        """
        check if the boat is within the threshold distance of any buoys
        marks the most recently reached buoy and increments its pass count
        
        args:
            current_position: tuple of (latitude, longitude) for current boat position
            buoys: list of waypoint objects representing buoys to check
            
        returns:
            list of waypoint objects that were newly reached during this check
        """
        newly_reached = []
        current_lat, current_lon = current_position
        
        for buoy in buoys:
            # calculate distance to buoy
            distance = calculate_distance(current_lat, current_lon, buoy.lat, buoy.long)
            
            # check if within threshold distance
            if distance <= self.threshold_distance:
                # This buoy has been reached
                if buoy not in self.detected_buoys or not buoy.marked:
                    # Either a new buoy or one we're passing again after passing others
                    buoy.n_passed += 1
                    newly_reached.append(buoy)
                    
                    # Unmark the previous current buoy
                    if self.current_buoy is not None and self.current_buoy != buoy:
                        self.current_buoy.marked = False
                    
                    # Mark this as the current buoy
                    buoy.marked = True
                    self.current_buoy = buoy
                    self.detected_buoys.add(buoy)  # track that we've seen this buoy
                
        return newly_reached
    
    def get_current_buoy(self) -> Optional[Waypoint]:
        """get the most recently passed buoy"""
        return self.current_buoy
    
    def is_buoy_reached(self, buoy: Waypoint) -> bool:
        """check if a specific buoy has been reached at least once"""
        return buoy.n_passed > 0
    
    def get_reached_buoys_count(self) -> int:
        """get the total number of buoys that have been reached at least once"""
        return len(self.detected_buoys)
    
    def reset(self):
        """reset the detector state"""
        self.detected_buoys.clear()
        self.current_buoy = None