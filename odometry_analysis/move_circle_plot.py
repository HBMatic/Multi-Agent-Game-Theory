import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import matplotlib.pyplot as plt


class RobotController:
    def __init__(self):
        rospy.init_node('move_circle', anonymous=True)

        # Publishers for both robots
        self.pub_real = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=100)
        self.pub_sim = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

        # Subscribers for odom data
        self.sub_real_odom = rospy.Subscriber('/tb3_3/odom', Odometry, self.real_odom_callback)
        self.sub_sim_odom = rospy.Subscriber('/odom', Odometry, self.sim_odom_callback)

        # Message counters
        self.cmd_vel_count = 0
        self.odom_real_count = 0
        self.odom_sim_count = 0

        # Odom data storage
        self.real_odom_data = {'x': [], 'y': [], 'theta': []}
        self.sim_odom_data = {'x': [], 'y': [], 'theta': []}

        self.rate = rospy.Rate(10)  # Increase the rate to 10 Hz

    def cmd_vel_callback(self):
        self.cmd_vel_count += 1

    def real_odom_callback(self, msg):
        self.odom_real_count += 1
        self.real_odom_data['x'].append(msg.pose.pose.position.x)
        self.real_odom_data['y'].append(msg.pose.pose.position.y)
        #self.real_odom_data['theta'].append(self.quaternion_to_euler(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

    def sim_odom_callback(self, msg):
        self.odom_sim_count += 1
        self.sim_odom_data['x'].append(msg.pose.pose.position.x)
        self.sim_odom_data['y'].append(msg.pose.pose.position.y)
        #self.sim_odom_data['theta'].append(self.quaternion_to_euler(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

    def move_circle(self, duration, linear_velocity=0.1, angular_velocity=0.1):
        move_cmd = Twist()
        move_cmd.linear.x = linear_velocity
        move_cmd.angular.z = angular_velocity

        start_time = time.time()
        rospy.loginfo("Starting to move in a circle")
        try:
            while not rospy.is_shutdown() and time.time() - start_time < duration:
                self.pub_real.publish(move_cmd)
                self.pub_sim.publish(move_cmd)
                self.cmd_vel_callback()  # Increment cmd_vel count
                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS Interrupt Exception! Stopping the robots.")
        finally:
            stop_cmd = Twist()
            self.pub_real.publish(stop_cmd)
            self.pub_sim.publish(stop_cmd)
            rospy.loginfo("Stopped the robots")
            rospy.loginfo(f"cmd_vel messages published: {self.cmd_vel_count}")
            rospy.loginfo(f"Odom messages read (real): {self.odom_real_count}")
            rospy.loginfo(f"Odom messages read (simulated): {self.odom_sim_count}")
            rospy.loginfo(f"Distance between two turtlebot: {self.calculate_distance_difference()}")
            self.plot_odom_data()
            
    def plot_odom_data(self):
        plt.figure(figsize=(10, 5))

        plt.subplot(1, 2, 1)
        plt.plot(self.real_odom_data['x'], self.real_odom_data['y'], label='Real Robot')
        plt.plot(self.sim_odom_data['x'], self.sim_odom_data['y'], label='Simulated Robot')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Trajectory')
        plt.legend()
        plt.grid(True) 

        #plt.subplot(1, 2, 2)
        #plt.plot(self.real_odom_data['theta'], label='Real Robot')
        #plt.plot(self.sim_odom_data['theta'], label='Simulated Robot')
        #plt.xlabel('Time Index')
        #plt.ylabel('Theta [rad]')
        #plt.title('Orientation Over Time')
        #plt.legend()
        #plt.grid(True) 

        plt.tight_layout()
        plt.show()
    
    def calculate_distance_difference(real_odom_data, sim_odom_data):
        real_x, real_y = real_odom_data['x'][-1], real_odom_data['y'][-1]
        sim_x, sim_y = sim_odom_data['x'][-1], sim_odom_data['y'][-1]

        distance_difference = math.sqrt((real_x - sim_x)**2 + (real_y - sim_y)**2)
        return distance_difference

if __name__ == '__main__':
    try:
        controller = RobotController()
        duration = 20 * 3.14159  # Example duration
        controller.move_circle(duration)
    except rospy.ROSInterruptException:
        pass
