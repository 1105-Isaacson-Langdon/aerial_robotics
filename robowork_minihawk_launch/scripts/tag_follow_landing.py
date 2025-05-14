#!/usr/bin/env python
import rospy
import time
import sys
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TwistStamped

class TagFollowLanding:
    def __init__(self):
        rospy.init_node('tag_follow_landing')

        #Publishers/Subscribers
        self.cmd_pub = rospy.Publisher(
            '/minihawk_SIM/mavros/setpoint_velocity/cmd_vel',
            TwistStamped, queue_size=10)
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
        if len(msg.detections) > 0:
            pose = msg.detections[0].pose.pose.pose.position
            self.tag_seen = True
	    if self.first_tag_time is None:
                self.first_tag_time = time.time()
                rospy.loginfo("First tag seen. Waiting before QLOITER...")


            #QLOITER switch
            if not self.qloiter_engaged and (time.time() - self.first_tag_time) > 5.0:
                self.set_mode('QLOITER')
                self.qloiter_engaged = True

            #PID-style control
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.twist.linear.x = -0.3 * pose.x
            cmd.twist.linear.y = -0.3 * pose.y
            cmd.twist.linear.z = -0.3 * pose.z - 0.2  #bias to descend

            self.cmd_pub.publish(cmd)

            rospy.loginfo("Tag Detected")
	    self.last_seen_time = time.time()

        else:
	    rospy.loginfo("No tags detected.")
            self.tag_seen = False

    def run(self):
        #Start in auto mode and arm
        self.set_mode('AUTO')
        self.arm()

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            #If tag was seen but lost, initiate landing
            if self.qloiter_engaged and not self.tag_seen:
                time_since_seen = time.time() - self.last_seen_time
                if time_since_seen > 3 and not self.landed:
                    rospy.loginfo("Tag lost, switching to QLAND")
                    self.set_mode('QLAND')
                    self.landed = True
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TagFollowLanding()
        node.run()
    except rospy.ROSInterruptException:
        pass
