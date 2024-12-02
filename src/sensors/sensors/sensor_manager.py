import rclpy
from rclpy.node import Node
# import custom messages


"""
A centeralized class to process and store sensor callbacks. 

All sensors should publish data to here.

Individual nodes should not be subscribed to multiple topics; they should query the manager
"""

class SensorManager(Node):
    def __init__(self):
        super().__init__('sensor_manager')

        # initialize storage for sensor data
        self.sensor_data = {
            'gps': None,
            'wind': None,
            'radio': None,
        }

        # subscriptions
        self.gps_subscription = self.create_subscription(

        )
        self.wind_subscription = self.create_subscription(

        )
        self.radio_subscription = self.create_subscription(

        )

    def gps_callback(self, msg):
        print("test")

    def wind_callback(self, msg):
        print("test")

    def radio_callback(self, msg):
        print("test")

    def get_sensor_data(self, sensor_type):
        return self.sensor_data.get(sensor_type, None)
    
# main entry point for the script
def main(args=None):
    rclpy.init(args=args)
    node = SensorManager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()