import rosbag
import tf

# Path to your ROS bag file
bag_file = './bags/BAG.bag'

# Open the ROS bag
bag = rosbag.Bag(bag_file)

# Specify the topics you want to extract data from
topics = ['/cmd_vel', '/pose']

# Iterate over the messages in the bag file
for topic, msg, t in bag.read_messages(topics=topics):
    
    # Format the timestamp with 8 digits before the comma and 8 digits after the comma
    timestamp = "{:0<8},{:0>8}".format(str(t.secs), str(t.nsecs))

    if topic == '/cmd_vel':
        # Process and print data from topic1
        print(f"Data from cmd_vel:\n {msg}")
        print(f"Timestamp: {timestamp}\n")

    elif topic == '/pose':
        # Process and print data from topic2
        print(f"Data from pose:\n {msg}")
        print(f"Timestamp: {timestamp}\n")
        
        # Extract the position data from the message
        position_x=msg.pose.pose.position.x
        position_y=msg.pose.pose.position.y
        
        # Extract the orientation data from the message
        orientation_x=msg.pose.pose.orientation.x
        orientation_y=msg.pose.pose.orientation.y
        orientation_z=msg.pose.pose.orientation.z
        orientation_w=msg.pose.pose.orientation.w
        
        covar_pose=msg.pose.covariance
        
        print("covariance: ", covar_pose)
        
        # Convert the orientation data to Euler angles     
        quarterion = (orientation_x, orientation_y, orientation_z, orientation_w)
        euler = tf.transformations.euler_from_quaternion(quarterion)
        
        # Extract roll, pitch, and yaw from Euler angles
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        # Print the Euler angles
        print("Roll:", roll)
        print("Pitch:", pitch)
        print("Yaw:\n", yaw)
        
    # Wait for user input before printing the next message
    input("Press Enter to continue...")

# Close the ROS bag
bag.close()