from path_planning.path_planning.waypoint import Waypoint

class Event:
    def initialize_event(self, boat):
        """
        base event class, all events inherit from this
        param boat: the boat instance to operate on
        """
        raise NotImplementedError("this method should be implemented in subclasses")

class F(Event):
    def initialize_event(self, boat):
        """fleet race event init"""
        print("initializing fleet race event type...")
        boat.state.autonomous_enabled = False
        boat.state.control_mode = boat.ControlMode.RC

class Pr(Event):
    def initialize_event(self, boat):
        """precision navigation event init"""
        print("initializing precision navigation event type...")
        boat.autonomous_system_initialized = True
        boat.state.autonomous_enabled = True
        print("autonomous system initialized for precision navigation")

        # define precision navigation waypoints
        boat.waypoints = [
            Waypoint(12.34, 56.78),
            Waypoint(13.45, 57.89),
            Waypoint(14.56, 58.90)
        ]
        boat.state.current_waypoint = boat.waypoints[0]
        print("waypoints loaded for precision navigation:", boat.waypoints)

class S(Event):
    def initialize_event(self, boat):
        """station keeping event init"""
        print("initializing station keeping event type...")
        boat.autonomous_system_initialized = True
        boat.state.autonomous_enabled = True
        print("autonomous system initialized for station keeping")

        boat.waypoints = [
            Waypoint(12.34, 56.78),
            Waypoint(13.45, 57.89),
            Waypoint(14.56, 58.90)
        ]
        boat.state.current_waypoint = boat.waypoints[0]

class E(Event):
    def initialize_event(self, boat):
        """endurance event init"""
        print("initializing endurance event type...")
        boat.autonomous_system_initialized = True
        boat.state.autonomous_enabled = True
        print("autonomous system initialized for endurance")

        boat.waypoints = [ # MAKE THIS 16 NOT 4 LATER
            Waypoint(12.34, 56.78),
            Waypoint(13.45, 57.89),
            Waypoint(14.56, 58.90),
            Waypoint(12.34, 56.78)
        ]
        boat.state.current_waypoint = boat.waypoints[0]

class P(Event):
    def initialize_event(self, boat):
        """payload event init"""
        print("initializing payload event type...")
        boat.autonomous_system_initialized = True
        boat.state.autonomous_enabled = True
        print("autonomous system initialized for payload")

        boat.waypoints = [
            Waypoint(12.34, 56.78),
            Waypoint(13.45, 57.89),
            Waypoint(14.56, 58.90)
        ]
        boat.state.current_waypoint = boat.waypoints[0]

class D(Event):
    def initialize_event(self, boat):
        """developer mode init"""
        print("initializing developer mode...")
        boat.autonomous_system_initialized = True
        boat.state.autonomous_enabled = True

        boat.waypoints = [
            Waypoint(12.34, 56.78),
            Waypoint(13.45, 57.89),
            Waypoint(14.56, 58.90)
        ]
        boat.state.current_waypoint = boat.waypoints[0]