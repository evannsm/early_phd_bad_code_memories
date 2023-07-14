from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from rosbagreader import 

# create reader instance and open for reading
with Reader('/home/ros/rosbag_2020_03_24') as reader:
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/imu_raw/Imu':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            print(msg.header.frame_id)

    # messages() accepts connection filters
    connections = [x for x in reader.connections if x.topic == '/imu_raw/Imu']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = deserialize_cdr(rawdata, connection.msgtype)
        print(msg.header.frame_id)