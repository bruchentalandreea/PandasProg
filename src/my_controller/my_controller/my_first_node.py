#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        """ This function is the constructor of Node named 'first_node' """
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(1.0,self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_+= 1 

def main(args=None):
    """ Initialise ros to communication """
    rclpy.init(args=args)
    """ Create a node """
    node = MyNode()
    """ Spin = the node is continue to run """
    rclpy.spin(node)
    """ Initialise ros to shutdown """
    rclpy.shutdown()

if __name__ == '__main__':
    main()