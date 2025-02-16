import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import gazebo_msgs.srv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.twist = Twist()
        
    def scan_callback(self, data):
        front_range = min(min(data.ranges[0:30] + data.ranges[-30:]), 10)
        left_range = min(min(data.ranges[30:90]), 10)
        right_range = min(min(data.ranges[-90:-30]), 10)
        
        if front_range < 0.5:  # Obstacle detected
            self.twist.linear.x = 0
            if left_range > right_range:
                self.twist.angular.z = 0.5  # Turn left
            else:
                self.twist.angular.z = -0.5  # Turn right
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
        
        self.vel_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass