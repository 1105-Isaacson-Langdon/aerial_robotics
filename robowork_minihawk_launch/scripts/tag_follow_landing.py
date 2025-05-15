#!/usr/bin/env python
import rospy
import time
import sys
import math
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TwistStamped

class TagFollowLanding:
    def __init__(self):
        rospy.init_node('tag_follow_landing')

        #Publishers/Subscribers
        self.cmd_pub = rospy.Publisher('/minihawk_SIM/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
        rospy.Subscriber('/minihawk_SIM/MH_usb_camera_link_optical/tag_detections', AprilTagDetectionArray, self.tag_callback)


        #Services
        rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
        rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')
        self.set_mode_srv = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        self.arm_srv = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)

        #State variables
        self.tag_seen = False
        self.last_seen_time = time.time()
        self.qloiter_engaged = False
        self.landed = False
	self.first_tag_time = None

    def set_mode(self, mode):
        try:
            response = self.set_mode_srv(0, mode)
            if response.mode_sent:
                rospy.loginfo("Mode set to {}".format(mode))
            else:
                rospy.logwarn("Failed to set mode to {}".format(mode))
        except rospy.ServiceException as e:
            rospy.logerr("Set mode failed: {}".format(e))

    def arm(self):
        try:
            self.arm_srv(True)
            rospy.loginfo("Vehicle armed")
        except rospy.ServiceException as e:
            rospy.logerr("Arming failed: {}".format(e))

    def tag_callback(self, msg):
        if msg.detections:
            d = msg.detections[0]
            p = d.pose.pose.pose.position
            q = d.pose.pose.pose.orientation

            #compute yaw error from quaternion
            yaw_err = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

            self.tag_seen = True
            now = time.time()
            if self.first_tag_time is None:
                self.first_tag_time = now
                rospy.loginfo("First tag seen; will enter QLOITER in 3 s")

            #after 3 s of tag seen engage QLOITER
            if not self.qloiter_engaged and (now - self.first_tag_time) > 3.0:
                self.set_mode('QLOITER')
                self.qloiter_engaged = True

            #publish velocity commands
            if self.qloiter_engaged:
                cmd = TwistStamped()
                cmd.header.stamp = rospy.Time.now()
                #gains
                cmd.twist.linear.x  = -0.3 * p.x
                cmd.twist.linear.y  = -0.3 * p.y
                cmd.twist.linear.z  = -0.3 * p.z - 0.2 #downward bias
                cmd.twist.angular.z = -0.5 * yaw_err #yaw correction

                self.cmd_pub.publish(cmd)
                rospy.loginfo(
                    "PID cmd  dx: %.2f, dy: %.2f, dz: %.2f, yaw_err: %.2f",
                    p.x, p.y, p.z, yaw_err
                )

            self.last_seen_time = now

        else:
            #lost detection
            if self.tag_seen:
                rospy.loginfo("Tag lost!")
            self.tag_seen = False

    def run(self):
        #start in auto mode and arm
        self.set_mode('AUTO')
        self.arm()

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            #if tag was seen but lost for 2 seconds, initiate landing
            if self.qloiter_engaged and not self.tag_seen:
                time_since_seen = time.time() - self.last_seen_time
                if not self.landed and time_since_seen > 2:
                    rospy.loginfo("Switching to QLAND")
                    self.set_mode('QLAND')
                    self.landed = True
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TagFollowLanding()
        node.run()
    except rospy.ROSInterruptException:
        pass
