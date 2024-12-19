def updateRudder(target):
        print("Called update rudder")
        print("Target angle is: " + target)

        # add serial communication between orin, arduino, and servo

        # this is only an interface method to talk to the sensor
        # this should use serial communication to send an integer target angle to the arduino
                # the arduino will handle time step conversion and oscillation management