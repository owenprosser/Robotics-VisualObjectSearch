import rospy, cv2, cv_bridge, numpy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

class robotsAssignment:
    def laser_call(self, data):
        global distance
        distance = min(data.ranges)
        #distance from object infront
        self.range = data.ranges[len(data.ranges)/2]
        #left side of screen (centre / 2)
        #right side of screen (centre * 2)
        
        self.distance = min(data.ranges)
        #distance from object infront calculation
        print "Distance from object %0.1f" % self.range
        
        
    def image_Red(self, msg):
        #TO SEE MASK#
        global MoveToRed
        global image
        global h, w ,d
        global cx, cy
        global red
        global lmao
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  
        #RED CORRECT VALUES
        lower_black = numpy.array([ 255,  255,  255])
        upper_black = numpy.array([255, 255, 255])        
        
        lower_red = numpy.array([ 0,  50,  50])
        upper_red = numpy.array([6, 255, 255])
        
        
        lower_Green = numpy.array([ 40,  100,  100])
        upper_Green = numpy.array([ 75, 255, 255])
        
        Mask = cv2.inRange(hsv, lower_black, upper_black)
        
        if (self.RedStop == True):
            Mask = cv2.inRange(hsv, lower_red, upper_red) 
            
        elif (self.SwapGreen == True):
              Mask = cv2.inRange(hsv, lower_Green, upper_Green)
              
        h, w, d = image.shape
        search_top = 1 * h/4
        search_bot = 3 * h/4
        Mask[0:search_top, 0:w] = 0
        Mask[search_bot:h, 0:w] = 0      
                        
        MoveToRed = cv2.moments(Mask)
        self.MoveToRed = cv2.moments(Mask)
        lmao = MoveToRed
        
        try:
            cx = int(MoveToRed['m10']/MoveToRed['m00'])
            cy = int(MoveToRed['m01']/MoveToRed['m00'])   
            
        except ZeroDivisionError:
            cx = 0
            cy = 0    
        
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("Red", 1)
        global RedStop
        global GreenStop
        global SwapGreen
        count = 0
        josh = 0
        spin_list = random.sample(range(-150,150),15)
        self.rate = rospy.Rate(2)
        
        self.publishTwist = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_Red)
        self.laser_sub = self.laser_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.laser_call)
        
        self.twist = Twist()
        self.moving = True
        self.RedStop = True
        self.GreenStop = True
        
        while not rospy.is_shutdown():
            if self.moving:
                if(distance > 1):
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = 0.0
                    self.publishTwist.publish(self.twist)
                    
                    if (MoveToRed > 0):    
                       if self.RedStop == True:
                           # BEGIN CONTROL
                           err = cx - w/2
                           self.twist.linear.x = 0.2
                           self.twist.angular.z = -float(err) / 100
                           self.publishTwist.publish(self.twist)
                           print "i see red"   
                           if(distance < 1.2):
                               print "i can no longer see red"
                               self.twist.angular.z = 4
                               self.twist.linear.x = 0
                               self.publishTwist.publish(self.twist)
                               self.RedStop = False
                               self.SwapGreen = True
                               self.GreenStop = True
                               
                       elif self.GreenStop == True:
                           err = cx - w/2
                           self.twist.linear.x = 0.2
                           self.twist.angular.z = -float(err) / 100
                           self.publishTwist.publish(self.twist)
                           print "i see Green"   
                           if(distance < 1.2):
                               print "i can no longer see Green"
                               self.twist.angular.z = 4
                               self.twist.linear.x = 0
                               self.publishTwist.publish(self.twist)
                               self.GreenStop = False

                        
                elif(distance < 1):
                    if ((count % 4) == 0):
                        josh = josh + 1
                
                    if (josh == 14):
                        josh = 0
                    
                    spin = float(spin_list[josh]/100.0)
                    self.twist.linear.x = 0
                    self.twist.angular.z = spin
                    self.publishTwist.publish(self.twist)
                
                    count = count + 1
                    print(spin)
                    self.rate.sleep()
                    
                else:
                    #print "lmao"
                    self.twist.angular.z = 1            
                    
distance = 1
MoveToRed = 10
image = 0
RedStop = 0
GreenStop = 0
SwapGreen = 0
h = 0
w = 0
d = 0
cx = 0
cy = 0
Red = 0
lmao = 0
rospy.init_node('robotAssignment', anonymous=True)
robotsAssignment = robotsAssignment()
rospy.spin()
