""" This node tells the robot to drive forward until a bumper is pressed"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class WallApproach(Node):
    """This is the wall approaching node, which inherits from the rclpy Node class."""
    def __init__(self):
        """ Initializes the Wall Approach Node. No Inputs."""
        super().__init__('wall_approach')                                   # Initializes the node and calls the node 'wall_approach'
        timer_period = 0.1                                                  # 0.1 symbolizes 10 cycles per second
        self.timer = self.create_timer(timer_period, self.run_loop)         # Create a timer, 0.1 is the period of the timer, we define run_loop as the callback function that is called every time the timer goes through one period
        self.publish_velocity = self.create_publisher(Twist, 'cmd_vel', 10) # Creates a publisher, Twist, which is a message used to convey linear and angular velocities. Sets the topic the publisher will publish the message to as "cmd_vel", and uses 10 as a value (use 10).

    def run_loop(self):
        """Executes the main logic for driving the square."""
        self.drive(linear_velocity=0.0)             # Executes the drive command, and commands the robot to drive with 0 linear velocity in the x direction
        self.drive(linear_velocity=0.1)             # Executes the drive command, and commands the robot to drive with 0.1 linear velocity in the x direciton
        print("driving")                            # Prints "driving" to tell us the robot was just commanded to drive 

    def drive(self, linear_velocity):
        """Drive with specified linear velocity
        
        Args:
            linear_velocity (float): the linear velocity in m/s
        """
        drive_cmd = Twist()                         # Give a twist message the name "drive_cmd"
        drive_cmd.linear.x = linear_velocity        # Sets the linear x value in the twist message as the inputted linear velocity argument
        self.publish_velocity.publish(drive_cmd)    # Publishes the Twist message out of the wall_approach node

def main(args=None):
    """Initializes the node, runs it, then cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """

    rclpy.init(args=args)   # Initialize communication with ROS
    node = WallApproach()   # Create our Node
    rclpy.spin(node)        # Run the Node until ready to shutdown
    node.destroy_node()     # Destroys the Node
    rclpy.shutdown()        # cleanup

if __name__ == '__main__':
    main()
