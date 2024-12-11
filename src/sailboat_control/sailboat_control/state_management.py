class Boat:
    def __init__(self, event_type: str):

        """
        Initialize the boat with a selected event type.

        :param event_type: The type of event (e.g... event names ->)
        """

        self.event_type = event_type
        self.autonomous_system_initialize = False

    def pick_event(self):
        """
        Pick and initialize the event based on the event type
        """

        event_classes = {

            "fleet_race": FleetRace,
            "precision_navigation": PrecisionNavigation,
            "station_keeping": StationKeeping,
            "endurance": Endurance,
            "payload": Payload

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

class FleetRace(Event):
    
    def initialize_event(self, boat: Boat):
        """
        Initialize the fleet race event, autonomous should never be activated
        """

        print("Initializing Fleet Race Event Type...")

class PrecisionNavigation(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize the precision navigation event      
        """

        print("Initializing The Precision Navigation Event Type")
        boat.autonomous_system_initialized = True
        print("Autonomous System Initialized For Precision Navigation")

class StationKeeping(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize the station keeping event
        """

        print("Initializing The Station Keeping Event Type")
        boat.autonomous_system_initializef = True
        print("Autonomous System Initialized For Station Keeping")

class Endurance(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize the endurance event
        """

        print("Initializing The Endurance Event Type")
        boat.autonomous_system_initializef = True
        print("Autonomous System Initialized For Endurance")

class Payload(Event):

    def initialize_event(self, boat: Boat):
        """
        Initialize the payload event
        """

        print("Initializing The Payload Event Type")
        boat.autonomous_system_initializef = True
        print("Autonomous System Initialized For Payload")