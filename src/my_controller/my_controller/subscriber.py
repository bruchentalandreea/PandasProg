#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
import argparse
from geometry_msgs.msg import Twist

class MySubscriber(Node):

    def __init__(self, topic:str):
        """ This function is the constructor of Node named 'subscriber' 
        Args: 
        param (str) : String with the name of the topic
        """
        super().__init__("subscriber")

        self.topic = topic
        self.data_frame = pd.DataFrame(columns=["TOPIC","LX", "LY", "LZ", "AX", "AY", "AZ"])

        """ Subscribe to a topic """
        self.subscriber = self.create_subscription(Twist,self.topic,self.listener_callback,10)

    def listener_callback(self, msg: Twist):
        """ This function is acting as a callback and is called everytime we recive a message 
        Args: 
        param (Twist) : Twist object recived 
        """
        self.get_logger().info('I heard: "%s"' % msg)
        new_row ={"TOPIC": self.topic, "LX": msg.linear.x, "LY": msg.linear.y, "LZ": msg.linear.z, "AX": msg.angular.x, "AY": msg.angular.y, "AZ": msg.angular.z}
        self.data_frame.loc[len(self.data_frame)] = new_row
        print(self.data_frame.to_string())       

def main(args=None):
    
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="add topic")
    args = parser.parse_args()
    my_subscriber = MySubscriber(str(args.input))
    rclpy.spin(my_subscriber)
    rclpy.shutdown()
