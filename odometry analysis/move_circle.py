import rospy
from geometry_msgs.msg import Twist
import time

def move_circle(duration):
    rospy.init_node('move_circle', anonymous=True)
    pub = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    linear_velocity = 0.2  # m/s
    angular_velocity = 0.5  # rad/s

    move_cmd = Twist()
    move_cmd.linear.x = linear_velocity
    move_cmd.angular.z = angular_velocity

    start_time = time.time()
    while not rospy.is_shutdown() and time.time() - start_time < duration:
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot after the duration
    stop_cmd = Twist()
    pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        duration = 20 * 3.14159  # Duration in seconds
        move_circle(duration)
    except rospy.ROSInterruptException:
        pass
