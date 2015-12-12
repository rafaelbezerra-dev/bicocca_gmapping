#!/usr/bin/env python
# license removed for brevity

# execute before running this code: chmod +x filename.py

#tf from the laser_link to base_link and from base_link to odom
import rospy, roslib, tf
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# sendTransform(position, quaternion, child_frame_id, parent_frame_id)

def getQuaternion(roll = 0, pitch = 0, yaw = 0):
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)

# def odom_to_map(msg):
#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y
#     w = msg.pose.pose.orientation.w
#     br = tf.TransformBroadcaster()
#     br.sendTransform((x, y, 0),
#                      tf.transformations.quaternion_from_euler(0, 0, w),
#                      rospy.Time.now(),
#                      'base_link',
#                      'odom')
def baselink_to_odom():
    # frame_id: odom
    # child_frame_id: base_footprint
    # transform:
    #   translation:
    #     x: 0.0
    #     y: 0.0
    #     z: 0.0
    #   rotation:
    #     x: 0.0
    #     y: 0.0
    #     z: 0.0
    #     w: 1.0
    br = tf.TransformBroadcaster()
    br.sendTransform((0.0, 0.0, 0.0),
                     (0.0, 0.0, 0.0, 1.0),
                     rospy.Time.now(),
                     'base_link',
                     'odom')

def odom_callback(msg):
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    pz = msg.pose.pose.position.z

    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w
    # rospy.loginfo('orientation[] x {}, y {}, z {}, w {}'.format(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

    br = tf.TransformBroadcaster()
    br.sendTransform((px, py, pz),
                     (ox, oy, oz, ow),
                     rospy.Time.now(),
                     'base_link',
                     'odom')

def baselaser_to_baselink():
    # frame_id: base_link
    # child_frame_id: base_laser
    # transform:
    #   translation:
    #     x: 0.275
    #     y: 0.0
    #     z: 0.15
    #   rotation:
    #     x: 0.0
    #     y: 0.0
    #     z: 0.0
    #     w: 1.0
    br = tf.TransformBroadcaster()
    br.sendTransform((0.0, 0.0, 0.0),
                     (0.0, 0.0, 0.0, 1.0),
                     rospy.Time.now(),
                     'base_laser',
                     'base_link')

def sickfront_to_baselaser():
    # SICK_FRONT (LMS291)
    # x = 80mm
    # y = 0mm
    # z = 450mm
    # Orientation: horizontal scanning plane, sensor pointing towards x = +infinite
    br = tf.TransformBroadcaster()
    br.sendTransform((0.08, 0.0, 0),
                     getQuaternion(yaw=0),
                     rospy.Time.now(),
                     'sick_front',
                     'base_laser')

def sickrear_to_baselaser():
    # SICK_REAR (LMS200)
    # x = -463mm
    # y = 1mm
    # z = 454mm
    # Orientation: horizontal scanning plane, sensor pointing towards x = -infinite
    br = tf.TransformBroadcaster()
    br.sendTransform((-0.463, 0.001, 0),
                     getQuaternion(yaw=np.deg2rad(180)),
                     rospy.Time.now(),
                     'sick_rear',
                     'base_laser')

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('odom',
                     Odometry,
                     odom_callback)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        sickfront_to_baselaser()
        sickrear_to_baselaser()
        baselaser_to_baselink()
        # baselink_to_odom()
        rate.sleep()

    rospy.spin()
