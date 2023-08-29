#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse
import random

class MyPublisher(Node):

    def __init__(self, topic:str):
        """ This function is the constructor of Node named 'my_publisher' 
        Args: 
        param (str) : String with the name of the topic
        """
        super().__init__("my_publisher")
        self.topic = topic

        """ Here we create the publisher """
        self.my_publisher_ = self.create_publisher(Twist, self.topic, 10)

        """ The timer is going to call the timer_callback function every 1 second after the node is created"""
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Publisher node has been started")
        self.i = 0.0

    def timer_callback(self):
        """ This function creates a msg object from the class Twist and publish it"""
        msg = Twist()

        msg.linear.x += self.i
        msg.angular.x += self.i + 1.0
        msg.linear.y += self.i + 2.0

        msg.angular.y += self.i + 3.0
        msg.linear.z += self.i + 4.0
        msg.angular.z += self.i + 5.0

        self.i += 1
        if self.i >= 30.0:
            self.i = 0.0

        """ Publish the msg """
        self.my_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input",help="topic name")
    args = parser.parse_args()

    my_publisher_ = MyPublisher(str(args.input))
    rclpy.spin(my_publisher_)
    rclpy.shutdown()
