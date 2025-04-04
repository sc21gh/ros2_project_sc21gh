import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal
from std_msgs.msg import String


class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10) #10 Hz
        self.subscription  # prevent unused variable warning
        self.sensitivity = 20
        self.blue_found = False
        self.publisher = self.create_publisher(String, 'blueFound', 10)
        self.movement_subscription = self.create_subscription(String, 'goToBlue', self.goToBlue_callback, 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.control = False
        self.oriented = False
        self.complete = False
        self.velocity = Twist()

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        
        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
    
        contours, hierarchy = cv2.findContours(blue_image,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        self.blue_found = False
        if len(contours) > 0:

            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100: 

                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y)) 
                radius = int(radius) 

                cv2.circle(image,center,radius,(255,255,0) ,1)

                # Then alter the values of any flags
                self.blue_found = True
                
                #if motion planning is still controlling movement, broadcast that blue is found
                if self.control == False:
                    #broadcast that blue is found 
                    msg = String()
                    msg.data = 'Blue Straight Ahead!'
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published: {msg.data}')
                
                #calculate how far the object is from the camera view centre.
                img_centre = int(image.shape[1] / 2) #get the centre of the image in x
                self.offset = center[0]-img_centre #get the x offset of the blue box
            
            else:
                self.blue_found = False   
                
        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.namedWindow('robot_feed',cv2.WINDOW_NORMAL) 
        cv2.imshow('robot_feed', image)
        cv2.resizeWindow('robot_feed',320,240)
        cv2.waitKey(3)

        #Check the robot is oriented
        if self.control == True and self.oriented == False:
            print("offset is ", self.offset)
            #check if facing the blue box
            if abs(self.offset) < 10: 
                    self.get_logger().info("Successfully oriented to the blue cube")
                    self.oriented = True
                    self.velocity.angular.z = 0.0 #stop turning
                    self.velocity.linear.x = 0.2 #start moving forward
                    self.twist_publisher.publish(self.velocity)
                    self.get_logger().info("Started movement to blue cube")


        #Check the robot is close enough to the blue box
        if self.control == True and self.oriented == True:
            print("size is ", cv2.contourArea(c))
            #stop moving when the contour area reaches a certain size
            if cv2.contourArea(c) > 275000:
                self.velocity.linear.x = 0.0 #stop moving
                self.twist_publisher.publish(self.velocity)
                self.get_logger().info("Blue cube reached successfully")
                self.complete = True



    def goToBlue_callback(self, msg): #this node now has movement control
        self.get_logger().info("Control recieved from motion planning module")
        self.control = True
        #start turning based on the offset to look at the blue box
        #offset had the side of the screen that the blue box is on
        self.velocity.angular.z = 0.0002 * -self.offset
        self.twist_publisher.publish(self.velocity)
        self.get_logger().info("Started orientation")


def main():
    def signal_handler(sig, frame):
        vision.stop()
        rclpy.shutdown()
    

    rclpy.init(args=None)
    vision = Vision()
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(vision,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if vision.complete == True:
                rclpy.shutdown()
    except ROSInterruptException:
        pass

    vision.destroy_node()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()


