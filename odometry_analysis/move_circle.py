import rospy
from geometry_msgs.msg import Twist
import time

def move_circle(duration, linear_velocity=0.1, angular_velocity=0.1):
    rospy.init_node('move_circle', anonymous=True)
    pub = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=100)
    rate = rospy.Rate(10)  # Increase the rate to 10 Hz

    move_cmd = Twist()
    move_cmd.linear.x = linear_velocity
    move_cmd.angular.z = angular_velocity

    start_time = time.time()
    rospy.loginfo("Starting to move in a circle")
    try:
        while not rospy.is_shutdown() and time.time() - start_time < duration:
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
        duration = 20 * 3.14159 + 2 # Example duration
        move_circle(duration)
    except rospy.ROSInterruptException:
        pass
