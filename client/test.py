#!/usr/bin/env python3

import rospy


def main():
    rospy.init_node('test_node', anonymous=True)
    rospy.loginfo("Test node Initialized")

    # Your OpenCV code here
    # For example


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass