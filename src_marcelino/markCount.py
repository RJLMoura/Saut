#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray

observed_ids = set()

def callback_fiducial(msg):

    for transform in msg.transforms:
        marker_id = transform.fiducial_id
        rospy.loginfo("Fiducial ID: {}".format(marker_id))

        if marker_id not in observed_ids:
            # Register the new ID
            observed_ids.add(marker_id)
            rospy.loginfo("New marker ID registered: {}".format(marker_id))

# Initialize the ROS node
rospy.init_node('subscriber_node')
new_data_flag = False
# Create a Subscriber object
sub2 = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback_fiducial)
rospy.spin()

total_markers = len(observed_ids)
print("Total number of markers observed:", total_markers)