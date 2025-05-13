#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

def takeoff():
    rospy.init_node('takeoff_node', anonymous=True)

    rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')
    rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
    rospy.wait_for_service('/minihawk_SIM/mavros/cmd/takeoff')

    try:
        arm = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)
        mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        takeoff_srv = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/takeoff', CommandTOL)

        mode_resp = mode(0, 'GUIDED')
        rospy.loginfo("Mode set to GUIDED: %s", mode_resp.mode_sent)

        arm_resp = arm(True)
        rospy.loginfo("Vehicle armed: %s", arm_resp.success)

        takeoff_resp = takeoff_srv(0, 0, 0, 0, 10)
        rospy.loginfo("Takeoff command sent: %s", takeoff_resp.success)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    takeoff()
