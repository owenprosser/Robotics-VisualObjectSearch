 #!/usr/bin/env python
import rospy, cv2, cv_bridge, time
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class Follower:
  distance = 0
  exploring = True

  foundRed = False
  foundGreen = False
  foundBlue = False
  foundYellow = False
  foundAll = False

  redBound = (np.array([ 0, 50, 50]), np.array([0, 255, 255]))
  greenBound = (np.array([ 30, 100, 0]), np.array([70, 255, 255]))
  blueBound = (np.array([ 100, 150, 50]), np.array([150, 255, 255]))
  yellowBound = (np.array([ 0, 100, 0]), np.array([5, 255, 255]))

  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #Setup Subscibers
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.laser_scan = rospy.Subscriber('/turtlebot/scan',LaserScan,self.laser_callback)
    #Setup Publishers
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    
    self.twist = Twist()

  def image_callback(self, msg):
    self.count = 0
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, w, d = image.shape

    redMask = cv2.inRange(hsv, self.redBound[0], self.redBound[1])
    greenMask = cv2.inRange(hsv, self.greenBound[0], self.greenBound[1])
    blueMask = cv2.inRange(hsv, self.blueBound[0], self.blueBound[1])
    yellowMask = cv2.inRange(hsv, self.yellowBound[0], self.yellowBound[1])
    redMoment = cv2.moments(redMask)
    greenMoment = cv2.moments(greenMask)
    blueMoment = cv2.moments(blueMask)
    yellowMoment = cv2.moments(yellowMask)

    cv2.imshow("blue mask", blueMask)
    cv2.imshow("red mask", redMask)

    if self.foundRed == False and redMoment['m00'] > 500:
      self.exploring = False
      cx = int(redMoment['m10']/redMoment['m00'])
      cy = int(redMoment['m01']/redMoment['m00'])
      err = cx - w/2
      self.twist.angular.z = -float(err) / 100
      if self.driveToObj() is True:
        self.checkOffCol(0)

    if self.foundBlue == False and blueMoment['m00'] > 500:
      cv2.imshow("blue mom",blueMoment['m00'])
      self.exploring = False
      cx = int(blueMoment['m10']/blueMoment['m00'])
      cy = int(blueMoment['m01']/blueMoment['m00'])
      err = cx - w/2
      self.twist.angular.z = -float(err) / 100
      if self.driveToObj() is True:
        self.checkOffCol(2)

    self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("Original Image",image)
    cv2.waitKey(3)

  def laser_callback(self, msg):
    Follower.distance = msg.ranges[10]
    Follower.distance = np.nanmean(msg.ranges)

  def driveToObj(self):
    #print(Follower.distance)
    if Follower.distance > 2:
      self.twist.linear.x = 0.5
      return False
    else:
      self.twist.linear.x = 0
      return True

  def checkOffCol(self, col):
    print(col)
    if col == 0:
      self.foundRed = True
      print("Red Complete")
      self.exploring = True
    elif col == 1:
      self.foundGreen = True
      print("Green Complete")
      self.exploring = True
    elif col == 2:
      self.foundBlue = True
      print("Blue Complete")
      self.exploring = True
    elif col == 3:
      self.foundYellow = True
      print("Yellow Complete")
      self.exploring = True
    else:
      return

if __name__ == "__main__":
  rospy.init_node('follower')
  follower = Follower()
  rospy.spin()