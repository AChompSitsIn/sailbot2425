class autopilot:

    [] # add primary autopilot control here. respectively source leg, updateRudder, and updateSails

    # leg -> returns intermediate point
    # updateRudder -> interfaces with arduino to update the rudder angle
    # updateSails -> interfaces with arduino to update the sail trim

    # additional logic + to do list:

        # subscribe to wind sensor and check for significant changes. if so call leg again
        