# ---------------- TO-DO LIST ---------------- 
# Add the predefined waypoints within each event class
# Implement the control() functionality
    # In tandem with this, evaluate how to handle RC control/interruptions
    # For full RC events (fleet race) I plan on making a second RC only control method to reduce the autonomous control method's complexity
    # For autonomous events with potential RC interrupts, the current plan will be to:
        # watch for incoming RC commands
        # upon receiving an RC command, control should return the most recently passed waypoint and forfeit control to RC commands
        # we will continue to use autonomous distance checks for passed buoys but also allow for manual intervention to mark a passed buoy
        # upon receiving a command to restart autonomous control, the boat will recalculate its path from its current position

        # This approach simplifies the ROS node structure at the cost of reducing the modularity of the control functions 
# Create a 'waypoint' class, which contains a waypoint's latitude, longitude, and a passed flag. passed will be set to true when the boat has passed it
    # This should allow for better management of which leg the boat is currently on
    # I expect to treat endurance as 16 individual waypoints, as opposed to 4 waypoints looped 4 times, as an attempt to improve course leg clarity and visualization

# The logic flow should be as follows:
    # boat object is defined with an event type -> waypoints are loaded into control -> boat is initially in RC control and awaits a signal to start autonomous ->
    # autonomous signal is received -> boat uses its current position to navigate to the first target and so on

    # in the event that the wind angle changes, we can re-call leg with the new wind to accommodate


class Boat:
    def __init__(self, event_type: str):

        """
        Initialize the boat with a selected event type.

        :param event_type: The type of event (e.g... event names ->)
        """

        self.event_type = event_type
        self.autonomous_system_initialize = False # we will need to modify this flag, planning on using a fundamental control() method which takes the event type and waypoints as parameters

    def pick_event(self):
        """
        Pick and initialize the event based on the event type
        """

        # Used a dictionary to make the setting of event types more obvious
        event_classes = {

            "fleet_race": F,
            "precision_navigation": Pr,
            "station_keeping": S,
            "endurance": E,
            "payload": P,
            "developer_mode": D

        }

        event_class = event_classes.get(self.event_type.lower())

        if event_class:
            event_instance = event_class()
            event_instance.initialize_event(self)
        else:
            print(f"Error: Event type '{self.event_type}' is not recognized.")

# base event class
class Event:

    def initialize_event(self, boat: Boat):
        """
        Initialize the event; this should always be overridden by child classes

        :param boat: the boat instance to operate on
        """

        raise NotImplementedError("This method should be implemented in subclasses")

class F(Event):
    
    def initialize_event(self, boat: Boat):
        """
        Initialize the fleet race event, autonomous should never be activated
        """

        print("Initializing Fleet Race Event Type...")

class Pr(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize the precision navigation event      
        """

        print("Initializing The Precision Navigation Event Type...")
        boat.autonomous_system_initialized = True
        print("Autonomous System Initialized For Precision Navigation")

class S(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize the station-keeping event
        """

        print("Initializing The Station Keeping Event Type...")
        boat.autonomous_system_initializef = True
        print("Autonomous System Initialized For Station Keeping")

class E(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize the endurance event
        """

        print("Initializing The Endurance Event Type...")
        boat.autonomous_system_initializef = True
        print("Autonomous System Initialized For Endurance")

class P(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize the payload event
        """

        print("Initializing The Payload Event Type...")
        boat.autonomous_system_initializef = True
        print("Autonomous System Initialized For Payload")

class D(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize developer mode
        """

        print("Initializing Developer Mode...")
