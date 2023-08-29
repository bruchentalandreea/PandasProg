import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rosbag2_py
from rclpy.serialization import serialize_message

class subscriberNode(Node):
    def __init__(self):
        super().__init__("Subscriber_with_rosbag")
        self.get_logger().info("Recording has started!")

        self.topic = "/topic"

        #Initializing bag writer
        self.writer = rosbag2_py.SequentialWriter()                        

        #Defining the bag
        storage_options = rosbag2_py._storage.StorageOptions(              
                                            uri="my_bag",
                                            storage_id='mcap'),
                                            
        converter_options = rosbag2_py._storage.ConverterOptions('', '')

        #Open the bag
        self.writer.open(storage_options, converter_options)                

        #Declaring the topic
        topic_info = rosbag2_py._storage.TopicMetadata(                     
                            name='/topic',
                            type='std_msgs/msg/String',
                            serialization_format='cdr')
        
        self.writer.create_topic(topic_info)

        #Initializing subscriber
        self.subscriber_ = self.create_subscription(String, self.topic, self.printMessage, 10)

        #Writing inside the bag
    def printMessage(self, msg: String):
        self.writer.write(                              
            '/topic',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        
def main(args = None):
    rclpy.init(args = args)

    node = subscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()