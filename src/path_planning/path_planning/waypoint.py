class Waypoint:
    def __init__(self, lat, long, marked=False, n_passed=0):
        self.lat = lat
        self.long = long
        self.marked = marked
        self.n_passed = n_passed

    def __repr__(self):
        return f"Waypoint(lat={self.lat}, long={self.long}, marked={self.marked}, n_passed={self.n_passed})"