import rosbag

# Path to your ROS bag file
bag_file = 'path/to/your/bag/file.bag'

# Open the ROS bag
bag = rosbag.Bag(bag_file)

# Specify the topics you want to extract data from
topics = ['/topic1', '/topic2']

# Iterate over the messages in the bag file
for topic, msg, t in bag.read_messages(topics=topics):
    # Process and print the message data
    print(f"Topic: {topic}")
    print(f"Message: {msg}")
    print(f"Timestamp: {t}\n")

# Close the ROS bag
bag.close()