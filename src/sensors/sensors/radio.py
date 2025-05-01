import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class DummyRadioNode(Node):
    def __init__(self):
        super().__init__('dummy_radio_node')
        
        # create publisher for radio commands
        self.publisher = self.create_publisher(
            Int32,
            'radio_commands',
            10
        )
        
        # create timer for checking input
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # print command options
        self.print_command_options()
        
    def timer_callback(self):
        try:
            # check if there's input available
            command = input()
            
            if command.lower() == 'h':
                self.print_command_options()
                return
                
            # try to convert input to integer
            command_int = int(command)
            
            # create and publish message
            msg = Int32()
            msg.data = command_int
            self.publisher.publish(msg)
            
            # log the command
            self.get_logger().info(f'Published command: {command_int}')
            
        except EOFError:
            # no input available s
            pass
        except ValueError:
            self.get_logger().warn('Please enter a valid integer command')
            self.print_command_options()
    
    def print_command_options(self):
        self.get_logger().info("""
Available commands:
0: Switch to RC control
1: Start autonomous mode
2: RC interrupt
3: Resume autonomous
4: Emergency stop
h: Show this help message
        """)

def main(args=None):
    rclpy.init(args=args)
    node = DummyRadioNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()