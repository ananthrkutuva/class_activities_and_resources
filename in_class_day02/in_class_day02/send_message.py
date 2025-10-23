""" This script explores publishing ROS messages in ROS using Python """
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped



class SendMessageNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendMessageNode. No Inputs."""
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(PointStamped, 'my_point', 10)     

    def run_loop(self):
        """Prints a message to the terminal"""
        my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        my_point = Point(x=1.0, y=2.0, z=0.0)
        my_point_stamped = PointStamped(header=my_header, point=my_point)
        self.publisher.publish(my_point_stamped)

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)       # Initialize communication with ROS
    node = SendMessageNode()    # Create our Node
    rclpy.spin(node)            # Run the Node until ready to shutdown
    rclpy.shutdown()            # cleanup

if __name__ == '__main__':
    main()