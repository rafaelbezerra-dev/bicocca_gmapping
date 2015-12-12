#!/usr/bin/env python
# license removed for brevity

# execute before running this code: chmod +x filename.py

import numpy as np
import rospy, roslib
from datetime import timedelta, datetime
from std_msgs.msg import String, Header
import tf

# Messages
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

__dataset__ = 'synced.csv'
__enable_logs__ = False
__odom_publisher__ = rospy.Publisher('odom', Odometry, queue_size=50)
__scan_publisher__ = rospy.Publisher('base_scan', LaserScan, queue_size=10)

def getOdometry(odometry_line):
    # Timestamp [seconds.microseconds]
    # Rolling Counter [signed 16bit integer]
    # TicksRight [ticks]
    # TicksLeft [ticks]
    # X [m]*
    # Y [m]*
    # Theta [rad]*
    # => 1235603339.32339, 4103, 2464, 2559, 1.063, 0.008, -0.028
    str_x, str_y, str_th = odometry_line.split(', ')[4:7]
    current_time = rospy.Time.now()
    th = float(str_th)
    x = float(str_x)
    y = float(str_y)
    z = 0.0
    roll = 0
    pitch = 0
    yaw = th
    qx, qy, qz, qw = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    # BUILDING ODOMETRY
    msg = Odometry()
    msg.header.stamp = current_time
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position = Point(x, y, z)
    msg.pose.pose.orientation = Quaternion(qx, qy, qz, qw)

    return msg

def getLaserScan(frame_id, laser_scan_line):
    # Timestamp [seconds.microseconds]
    # # of ranges [unitless]
    # Angular offset [1/4 degree]
    # R1..R181 Ranges (zero padded to 181 ranges) [m]
    #
    # 1235603336.30835, 181, 0, 11.360, 11.360, 11.390, 11.410, 81.910, 81.910, 11.380, 11.400, 11.430, 6.450, 6.170, 6.030, 5.880, 5.740, 5.600, 5.470, 5.360, 5.370, 5.390, 5.430, 5.470, 5.500, 5.530, 5.580, 5.610, 5.410, 5.230, 5.130, 5.180, 5.230, 5.280, 5.350, 6.040, 6.110, 6.180, 6.250, 6.330, 6.400, 6.490, 5.950, 5.750, 5.640, 5.520, 5.440, 5.330, 5.220, 5.280, 5.040, 5.490, 5.590, 5.690, 5.810, 5.930, 6.080, 6.210, 6.360, 6.530, 6.690, 6.870, 13.930, 13.770, 13.650, 13.650, 13.530, 13.430, 13.300, 13.190, 13.040, 12.870, 12.780, 12.700, 12.630, 12.550, 12.480, 12.410, 12.360, 12.310, 12.240, 12.200, 12.150, 12.110, 12.070, 12.040, 12.010, 11.990, 11.970, 11.560, 11.930, 11.920, 11.920, 11.910, 11.930, 11.920, 11.920, 11.940, 11.930, 12.830, 12.840, 12.300, 12.130, 12.120, 13.000, 12.250, 12.230, 12.270, 12.330, 12.390, 12.440, 12.520, 12.580, 12.810, 13.640, 13.740, 13.830, 13.940, 13.640, 6.410, 6.220, 6.010, 5.810, 5.640, 5.080, 4.180, 4.090, 4.250, 4.070, 4.050, 3.700, 3.560, 3.510, 3.510, 3.570, 3.430, 3.520, 3.590, 4.940, 4.650, 4.630, 5.050, 5.040, 5.080, 4.890, 2.790, 2.710, 2.660, 2.620, 2.590, 2.600, 2.660, 2.650, 2.630, 2.690, 2.790, 2.900, 4.250, 4.150, 2.510, 2.480, 2.390, 2.360, 2.330, 2.320, 2.300, 2.410, 2.270, 3.930, 2.290, 2.390, 3.850, 3.830, 3.830, 3.710, 4.060, 4.050, 4.040, 4.030, 4.020, 4.010, 4.010, 4.010, 4.010
    str_timestamp, str_num_readings, str_angular_offset, str_ranges = laser_scan_line.split(', ', 3)
    num_readings = int(str_num_readings)
    angular_offset = float(str_angular_offset)
    ranges = map(float, str_ranges.split(', ')) #convert array of readings
    laser_frequency = 50
    angle_range_rad = 180 * np.pi / 180

    #populate the LaserScan message
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.angle_min = - (angle_range_rad / 2)
    msg.angle_max = (angle_range_rad / 2)
    msg.angle_increment = angle_range_rad / num_readings
    msg.time_increment = (1 / laser_frequency) / (num_readings)
    msg.range_min = 0.0
    msg.range_max = 40.0
    msg.ranges = ranges
    msg.intensities = [0.0] * len(ranges)

    return msg

def publishOdometry(line):
    msg = getOdometry(line)

    # PUBLISHES ODOMETRY MENSAGE
    __odom_publisher__.publish(msg)

    if __enable_logs__:
        t = str(msg.header.stamp)
        x = str(msg.pose.pose.position.x)
        y = str(msg.pose.pose.position.y)
        w = str(msg.pose.pose.orientation.w)
        rospy.loginfo('Sent ODOM: {}   {}  {}  {}'.format(t, x, y, w))

def publishLaserScan(frame_id, line):
    msg = getLaserScan(frame_id, line)
    __scan_publisher__.publish(msg)

    if __enable_logs__:
        rospy.loginfo('Sent {}: {}'.format(frame_id.upper(), msg.ranges[:5]))



def publisher():
    rospy.init_node('data_publisher', anonymous=True)
    rate = rospy.Rate(20)
    rospy.loginfo('PUBLISHING...')

    f = open(__dataset__, 'r')


    counter = 1

    while not rospy.is_shutdown():
        # if __enable_logs__:
        #     rospy.loginfo('Message #' + str(counter))

        # ODOMETRY
        line = f.readline()
        if line == '':
            break
        header, msg = line.split(', ',1)
        if header == 'ODOMETRY':
            publishOdometry(msg)
        elif header == 'SICK_FRONT':
            publishLaserScan(header.lower(), msg)
        elif header == 'SICK_REAR':
            publishLaserScan(header.lower(), msg)
        else:
            pass
        rospy.loginfo('Publishing {} of 346838 '.format(counter))
        counter += 1
        rate.sleep()
    f.close()
    rospy.loginfo('DONE')

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
