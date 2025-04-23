import rospy
from geometry_msgs.msg import Twist
import time

def move_circle():
    rospy.init_node('line', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    rate = rospy.Rate(10)  # Increase the rate to 10 Hz

    move_cmd = Twist()
    move_cmd.linear.x = 0.1
    move_cmd.angular.z = 0

    start_time = time.time()
    rospy.loginfo("Starting to move in a line")
    try:
        while not rospy.is_shutdown() and time.time() - start_time < 20:
            pub.publish(move_cmd)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception! Stopping the robot.")
    finally:
        # Stop the robot after the duration
        stop_cmd = Twist()
        pub.publish(stop_cmd)
        rospy.loginfo("Stopped the robot")

if __name__ == '__main__':
    try:
        move_circle()
    except rospy.ROSInterruptException:
        pass
