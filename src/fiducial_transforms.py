import rospy
from geometry_msgs.msg import TransformStamped

def callback(msg):
    # Process the received message here
    # You can access the transform data using msg.transform

    # Print the message
    print(msg)

#Initialize the ROS node
rospy.init_node('fiducial_transforms_listener')

#Create a Subscriber object to subscribe to the topic
sub = rospy.Subscriber('/fiducial_transforms', TransformStamped, callback)

#Spin the node to receive messages
rospy.spin()