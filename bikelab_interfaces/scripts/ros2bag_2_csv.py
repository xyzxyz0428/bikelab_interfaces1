import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import re

def read_bag(bag_path):
    # Set up storage and converter options
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    # Create a reader instance and open the bag file
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get the list of topics and their types
    topics = reader.get_all_topics_and_types()

    # Create a dictionary to map topic names to their message types
    topic_types = {topic.name: topic.type for topic in topics}

    # Create a dictionary to hold dataframes for each topic
    topic_dataframes = {}

    # Read messages from each topic
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in topic_dataframes:
            topic_dataframes[topic] = []

        # Get the message type for the topic
        msg_type = get_message(topic_types[topic])

        # Deserialize the message
        msg = deserialize_message(data, msg_type)

        # Convert the message to a dictionary and append to the list
        topic_dataframes[topic].append(msg_to_dict(msg))

    # Convert lists of dictionaries to pandas DataFrames
    for topic in topic_dataframes:
        topic_dataframes[topic] = pd.DataFrame(topic_dataframes[topic])

    return topic_dataframes

def msg_to_dict(msg):
    # Convert ROS 2 message to dictionary
    msg_dict = {}
    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field)
        # Handle the header field separately
        if field == 'header':
            msg_dict['header.stamp.sec'] = value.stamp.sec
            msg_dict['header.stamp.nanosec'] = value.stamp.nanosec
            msg_dict['header.frame_id'] = value.frame_id
        else:
            msg_dict[field] = value
    return msg_dict

def sanitize_sheet_name(name):
    # Replace invalid Excel sheet name characters with underscores
    return re.sub(r'[\[\]:*?/\\]', '_', name)

def save_to_excel(dataframes, output_file):
    # Create a Pandas Excel writer using XlsxWriter as the engine
    with pd.ExcelWriter(output_file, engine='xlsxwriter') as writer:
        for sheet_name, df in dataframes.items():
            # Sanitize the sheet name
            sanitized_sheet_name = sanitize_sheet_name(sheet_name)
            # Write each dataframe to a different worksheet
            df.to_excel(writer, sheet_name=sanitized_sheet_name, index=False)

def main():
    bag_path = '/media/bikelab/KESU/assessment/0819/morning_A/rosbag2_2025_08_19-08_33_30/rosbag2_2025_08_19-08_33_30_0.db3'  # Update this path
    output_file = '/media/bikelab/KESU/assessment/0819/morning_A/rosbag2_2025_08_19-08_33_30/rpi.xlsx'  # Update this path

    # Read the bag file
    topic_dataframes = read_bag(bag_path)

    # Save the data to an Excel file
    save_to_excel(topic_dataframes, output_file)

if __name__ == '__main__':
    main()
