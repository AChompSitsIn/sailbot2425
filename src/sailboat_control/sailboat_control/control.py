class autopilot:

    [] # add primary autopilot control here. respectively source leg, updateRudder, and updateSails

    # leg -> returns intermediate point
    # updateRudder -> interfaces with arduino to update the rudder angle
        # updateRudder should have inherent handling for time step conversions to prevent oscillations (see henri rudder constant documentation)
    # updateSails -> interfaces with arduino to update the sail trim
        # updateSails should also have inherent handling for time step conversions (lower oscillation risk than rudder updates)

    # additional logic + to do list:

        # subscribe to wind sensor and check for significant changes. if so call leg again
        