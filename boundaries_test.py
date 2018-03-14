 #!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class Follower:
  distance = 0
  #Red, Blue, Geen, Yellow
  found = [False,False,False,False]
  seeColour = False

  boundaries = [
    ([109, 109, 109], [ 73, 73, 73]),
    ([ 0, 50, 50], [0, 255, 255])
  ]

# ([ 25, 100, 50], [255, 255, 255] ),

  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    #Setup Subscibers
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.laser_scan = rospy.Subscriber('/turtlebot/scan',LaserScan,self.laser_callback)
    #Setup Publishers
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape

    for (lower, upper) in self.boundaries:
      lower = numpy.array(lower, dtype= 'uint8')
      upper = numpy.array(upper, dtype= 'uint8')
      mask = cv2.inRange(hsv, lower, upper)
      cv2.imshow('mask',mask)
      M = cv2.moments(mask)
      if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.5
        self.twist.angular.z = -float(err) / 100
        if(Follower.distance < 1):
          self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
    
    print(self.seeColour)
    cv2.imshow("Original Image",image)
    cv2.waitKey(3)

  def laser_callback(self, msg):
    Follower.distance = msg.ranges[10]
    Follower.distance = numpy.nanmean(msg.ranges)
    print(str(Follower.distance))


if __name__ == "__main__":
  rospy.init_node('follower')
  follower = Follower()
  rospy.spin()
  # END ALL
