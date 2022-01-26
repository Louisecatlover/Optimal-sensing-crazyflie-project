#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
from crazyflie_driver.srv import UpdateParams

def onNewTransform(pose):
    global msg
    global pub
    global firstTransform

    if firstTransform:
	rospy.set_param("stabilizer/estimator", 2)
        update_params(["stabilizer/estimator"])
        # initialize kalman filter
        rospy.set_param("kalman/initialX", pose.pose.position.x)
        rospy.set_param("kalman/initialY", pose.pose.position.y)
        rospy.set_param("kalman/initialZ", pose.pose.position.z)
        update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
	rospy.set_param("kalman/resetEstimation", 1)
        update_params(["kalman/resetEstimation"])
        firstTransform = False
    else:
        msg.header.frame_id = pose.header.frame_id
        msg.header.stamp = pose.header.stamp
        msg.header.seq += 1
        msg.pose.position.x = pose.pose.position.x
        msg.pose.position.y = pose.pose.position.y
        msg.pose.position.z = pose.pose.position.z
        msg.pose.orientation.x = pose.pose.orientation.x
        msg.pose.orientation.y = pose.pose.orientation.y
        msg.pose.orientation.z = pose.pose.orientation.z
        msg.pose.orientation.w = pose.pose.orientation.w
        pub.publish(msg)



rospy.init_node('publish_external_position_vrpn', anonymous=True)
topic = rospy.get_param("~topic", "/crazyflie1/vrpn_client_node/crazyflie1/pose")

rospy.wait_for_service('update_params')
rospy.loginfo("found update_params service")
update_params = rospy.ServiceProxy('update_params', UpdateParams)

firstTransform = True

msg = PoseStamped()
msg.header.seq = 0
msg.header.stamp = rospy.Time.now()
rate = rospy.Rate(120)
pub = rospy.Publisher("external_pose", PoseStamped, queue_size=1)
rospy.Subscriber(topic, PoseStamped, onNewTransform)

while not rospy.is_shutdown():
    rate.sleep()
