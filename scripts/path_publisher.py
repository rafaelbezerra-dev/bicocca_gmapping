#!/usr/bin/env python
import rospy, roslib
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


__path_publisher__ = rospy.Publisher('path', Path, queue_size=50)
__poses__ = []
__enable_logs__ = False

def callback(odometry):
    stampedPose = PoseStamped()
    stampedPose.pose = odometry.pose.pose
    __poses__.append(stampedPose)

    msg = Path()
    msg.header.frame_id = 'odom'
    msg.header.stamp = rospy.Time.now()
    msg.poses = __poses__
    __path_publisher__.publish(msg)

    if __enable_logs__:
        rospy.loginfo('Sent PATH: \n{}'.format(odometry.pose.pose))

def listener():
    topic = 'odom'
    rospy.init_node('path_publisher', anonymous=True)
    rospy.loginfo("Listening to {}...".format(topic))

    rospy.Subscriber(topic, Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
