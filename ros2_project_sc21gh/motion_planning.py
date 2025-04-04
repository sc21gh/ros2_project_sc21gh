import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from math import sin, cos
import math
import random
import time
from std_msgs.msg import String
from rclpy.exceptions import ROSInterruptException

class NavigationGoalActionClient(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        self.complete = False
        self.get_logger().info("Sending coordinates ({},{})".format(x,y))
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.complete = True
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.complete = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #print(feedback)

class Spin(Node):
    def __init__(self):
        super().__init__('spin')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) # 10hz
        self.subscription = self.create_subscription(String, 'blueFound', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.blue_found = False
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Recieved: {msg.data!r}') #blue straight ahead
        self.blue_found = True

    def spin_360(self):
        self.get_logger().info("Starting 360 spin")
        self.complete = False
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.callback) #timer runs every 0.1 seconds
    
    def callback(self):
        desired_velocity = Twist()
        time_elapsed = time.time() - self.start_time
        if time_elapsed < 8 and self.blue_found == False:
            desired_velocity.angular.z = 0.785 #radians per second to spin 360 degrees n 8 seconds
            self.publisher.publish(desired_velocity)
        else:
                desired_velocity.angular.z = 0.0
                self.publisher.publish(desired_velocity)
                self.complete = True
                self.get_logger().info("Finished 360 spin")


def main(args=None):
    rclpy.init(args=args)
    navigation_goal_action_client = NavigationGoalActionClient()
    spinner = Spin()
    
    try:
        while rclpy.ok():
                found = False
                while (found==False):
                    x = round(random.uniform(-11,9),2) #x range from 9 to -11
                    y = round(random.uniform(-15,6),2) #y range from 6 to -15

                    #travel to the random coordinate
                    navigation_goal_action_client.send_goal(x, y, 0.0)  # example coordinates
                    while navigation_goal_action_client.complete == False:
                        rclpy.spin_once(navigation_goal_action_client)

                    #spin 360 degrees to look for the blue cube
                    spinner.spin_360()
                    while spinner.complete == False:
                        rclpy.spin_once(spinner)
                    
                    if spinner.blue_found: #if blue is found 
                        found = True
                        break

                #publish message to vision node that it can now go to the blue cube
                publisherNode = Node("publisher")
                publisherNode.publisher = publisherNode.create_publisher(String, 'goToBlue', 10)
                msg = String()
                msg.data = 'Control passed to vision node'
                publisherNode.publisher.publish(msg)

                print("Motion planning finished")
                navigation_goal_action_client.destroy_node()
                spinner.destroy_node()
                publisherNode.destroy_node()
                rclpy.shutdown()
                
    except ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
