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

        # Publishers/Subscribers
        self.cmd_pub = rospy.Publisher(
            '/minihawk_SIM/mavros/setpoint_velocity/cmd_vel',
            TwistStamped, queue_size=10)
        rospy.Subscriber(
            '/minihawk_SIM/MH_usb_camera_link_optical/tag_detections',
            AprilTagDetectionArray, self.tag_callback)

        # Service proxies
        rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
        rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')
        self.set_mode_srv = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        self.arm_srv = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)

        # State variables
        self.tag_seen = False
        self.last_seen_time = time.time()
        self.qloiter_engaged = False
        self.landed = False
        self.centered_start_time = None

        # PID gains
        self.Kp_x = 0.5
        self.Kp_y = 0.5
        self.Kp_z = 0.4
        self.descent_bias = 0.1  # gentle descent

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
        rospy.loginfo_throttle(2, "tag_callback triggered. Detections: {}".format(len(msg.detections)))

        if len(msg.detections) > 0:
            pose = msg.detections[0].pose.pose.pose.position
            rospy.loginfo("Pose values: x={:.2f}, y={:.2f}, z={:.2f}".format(pose.x, pose.y, pose.z))

            self.tag_seen = True
            self.last_seen_time = time.time()

            # Check if centered in X and Y
            centered = abs(pose.x) < 0.1 and abs(pose.y) > -3.1
            if centered:
                if self.centered_start_time is None:
                    self.centered_start_time = time.time()
                elif time.time() - self.centered_start_time > 1.0 and not self.qloiter_engaged:
                    rospy.loginfo("Centered and stable. Switching to QLOITER.")
                    self.set_mode('QLOITER')
                    self.qloiter_engaged = True
            else:
                self.centered_start_time = None

            # Tag control once in QLOITER
            if self.qloiter_engaged and not self.landed:
                cmd = TwistStamped()
                cmd.header.stamp = rospy.Time.now()
                cmd.twist.linear.x = -self.Kp_x * pose.x
                cmd.twist.linear.y = -self.Kp_y * pose.y
                cmd.twist.linear.z = -self.Kp_z * pose.z - self.descent_bias

                self.cmd_pub.publish(cmd)

                rospy.loginfo("Velocity: x={:.2f}, y={:.2f}, z={:.2f}".format(
                    cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z))

                # Landing trigger (if centered and below 1m)
                if abs(pose.x) < 0.1 and abs(pose.y) < 0.1 and pose.z < 1.0:
                    rospy.loginfo("Centered and low. Switching to QLAND.")
                    self.set_mode('QLAND')
                    self.landed = True

        else:
            self.tag_seen = False
            self.centered_start_time = None
            rospy.loginfo_throttle(5, "No tags detected.")

    def run(self):
        self.set_mode('AUTO')
        self.arm()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.qloiter_engaged and not self.tag_seen:
                time_since_seen = time.time() - self.last_seen_time
                if time_since_seen > 3 and not self.landed:
                    rospy.loginfo("Tag lost for {:.1f} sec, switching to QLAND".format(time_since_seen))
                    self.set_mode('QLAND')
                    self.landed = True
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TagFollowLanding()
        node.run()
    except rospy.ROSInterruptException:
        pass
