#!/usr/bin/env python3
import pandas as pd
import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from geometry_msgs.msg import Twist

# Reading messages from the MCAP file
def read_messages(input_bag: str):
    first = "-1"

    reader = rosbag2_py.SequentialReader()
    # Open the bag
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    # Read topics from bag
    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
             return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        frame = int((timestamp % (10 ** 14)) / (10 ** 9))

        if first == "-1":
            first = frame

        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, frame - first
    del reader

def main(args=None):
    df = pd.DataFrame(columns=["FRAME", "TOPIC", "LX", "LY", "LZ", "AX", "AY", "AZ"])

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    # Add arguments lists for topic,frame,column
    parser.add_argument('-t', '--topic', action='append', help='delimited list input', type=str)
    parser.add_argument('-f', '--frame', action='append', help='delimited list input', type=str)
    parser.add_argument('-c', '--column', action='append', help='delimited list input', type=str)

    # Cmnd ex: python3 src/my_controller/my_controller/rosbag_to_pd.py my_bag -t /topic,/topic -f 32,12 -c AX,LZ
    args = parser.parse_args()

    # Iterate through the elements from MCAP, convert to pandas dataframe and display it 
    for topic, msg, frame in read_messages(args.input):
        if isinstance(msg, Twist):
            new_row =  {"FRAME": frame, "TOPIC": topic, "LX": msg.linear.x, "LY": msg.linear.y, "LZ": msg.linear.z, "AX": msg.angular.x, "AY": msg.angular.y, "AZ": msg.angular.z}
            df.loc[len(df)] = new_row
            #print(df.head())
            print(f"[{frame}] {topic}: '{msg.linear.x}','{msg.linear.y}','{msg.linear.z}','{msg.angular.x}','{msg.angular.y}', '{msg.angular.z}'")
        else:
            print(f"[{frame}] {topic}: ({type(msg).__name__})")
            print("Error")

    list_top = []
    list_frm = []
    list_clm = []
    
    # Covert the arguments from command line to list
    for top in args.topic:
        for i in top.split(','):
            list_top.append(i)

    for frm in args.frame:
        for i in frm.split(','):
            list_frm.append(i)
    
    for clm in args.column:
        for i in clm.split(','):
            list_clm.append(i)
    
    # Display the value for each topic, frame and column given zip(list_top,list_frm,list_clm)
    try:
        for (top,frm,clm) in zip(list_top,list_frm,list_clm):
            print(f"\n Search result: {float(df.loc[(df['FRAME'] == int(frm)) & (df['TOPIC'] == str(top)), clm].values)},", f" Topic: {top}, Frame: {frm}, Column: {clm}\n")
            print ( "------------------------------------------------------------------")

    except:
          print(f"[ERROR] Invalid column!: ({type(clm).__name__}) - ({args.topic}), ({args.frame}), ({args.column}) ")

if __name__ == '__main__':
    main()
