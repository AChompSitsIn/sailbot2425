class boat:
    print("boat initialized")
    
    # event types
    fleet_race = False
    station_keeping = False
    precision_navigation = False
    endurance = False
    payload = False

    if(fleet_race):
        print("fleet race event type initialized")
    
    if(station_keeping):
        print("station keeping event type initialized")
    
    if(precision_navigation):
        print("precision navigation event type initialized")

    if(endurance):
        print("endurance event type initialized")

    if(payload):
        print("payload race event type initialized")

class fleet_race:

    print("fleet race initialized, autonomous navigation is set to false by default")
    waypoints = None
    autonomous = False

class station_keeping:

    print("staton keeping initialized, autonomous navigation is set to false by default")
    waypoints = []
    autonomous = False

class precision_navigation:

    print("precision navigation initialized, autonomous navigation is set to false by default")
    waypoints = []
    autonomous = False

class endurance:

    print("endurance initialized, autonomous navigation is set to false by default")
    waypoints = []
    autonomous = False

class payload:

    print("payload initialized, autonomous navigation is set to false by default")
    waypoints = []
    autonomous = False
