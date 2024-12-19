class Waypoint:
    def __init__(self, lat, long, marked=False):
        self.lat = lat
        self.long = long
        self.marked = marked

    def __repr__(self):
        return f"Waypoint(lat={self.lat}, long={self.long}, marked={self.marked})"