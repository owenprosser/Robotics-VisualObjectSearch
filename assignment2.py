#!/usr/bin/env python
import rospy, cv2, cv_bridge, time
import numpy as np
from sensor_msgs.msg import Image, LaserScan, PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped


class Follower:
    distance = 10
    acml = []
    exploring = True

    foundRed = False
    foundGreen = False
    foundBlue = False
    foundYellow = False
    foundAll = False

    redBound = (np.array([ 0, 50, 50]), np.array([0, 255, 255]))
    greenBound = (np.array([ 50, 150, 50]), np.array([100, 255, 255]))
    blueBound = (np.array([ 100, 150, 50]), np.array([150, 255, 255]))
    yellowBound = (np.array([ 30, 200, 100]), np.array([50, 255, 195]))

    positions = [
        #   x,     y,   visited
        [ -2.29, -4.10, 0],
        [ -1.50, -4.10, 0],
        [ -0.45, -0.67, 0],
        [ -1.50, -4.10, 0],
        [ -1.74, 1.90,  0],
        [0.60, 4.70, 0],
        [-4.10, 1.70, 0]
    ]

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #Setup Subscibers
        self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)

        self.laser_scan = rospy.Subscriber('/turtlebot/scan',LaserScan,self.laser_callback)
        
        #Setup Publishers
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=10)

        self.position_sub = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.update_amcl_data)

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
        allMasks = redMask + blueMask + yellowMask + greenMask

        cv2.imshow("all masks", allMasks)
        self.checkComplete()

        if self.foundAll == False:
            if self.foundRed == False and cv2.countNonZero(redMask) > 0:
                self.exploring = False
                cx = int(redMoment['m10']/redMoment['m00'])
                cy = int(redMoment['m01']/redMoment['m00'])
                err = cx - w/2
                self.twist.angular.z = -float(err) / 100
                dist = (redMoment['m10'] / 100000000)
                if self.driveToObj(dist) is True:
                    self.checkOffCol(0)
            if self.foundGreen == False and cv2.countNonZero(greenMask) > 0:
                self.exploring = False
                cx = int(greenMoment['m10']/greenMoment['m00'])
                cy = int(greenMoment['m01']/greenMoment['m00'])
                err = cx - w/2
                self.twist.angular.z = -float(err) / 100
                dist = (greenMoment['m10'] / 100000000)
                if self.driveToObj(dist) is True:
                    self.checkOffCol(1)
            if self.foundBlue == False and cv2.countNonZero(blueMask) > 0:
                self.exploring = False
                cx = int(blueMoment['m10']/blueMoment['m00'])
                cy = int(blueMoment['m01']/blueMoment['m00'])
                err = cx - w/2
                self.twist.angular.z = -float(err) / 100
                dist = (blueMoment['m10'] / 100000000)
                if self.driveToObj(dist) is True:
                    self.checkOffCol(2)
            if self.foundYellow == False and cv2.countNonZero(yellowMask) > 0:
                self.exploring = False
                cx = int(yellowMoment['m10']/yellowMoment['m00'])
                cy = int(yellowMoment['m01']/yellowMoment['m00'])
                err = cx - w/2
                self.twist.angular.z = -float(err) / 100
                dist = (yellowMoment['m10'] / 100000000)
                if self.driveToObj(dist) is True:
                    self.checkOffCol(3)
        else:
            print("Found all Coloured Poles")

        self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("Original Image",image)
        cv2.waitKey(3)

    def search(self):
        if self.exploring == True:
            print("need to search")

    def rotate(self):
        start = self.amcl.pose.pose.orientation
        self.twist.angular.z = 1
        if self.amcl.pose.pose.orientation == start:
            self.twist.angular.z = 1
    
    def update_amcl_data(self, data):
        self.amcl = data

    def laser_callback(self, msg):
        #Follower.distance = min(msg.ranges)
        Follower.distance = msg.ranges[len(msg.ranges)/2]

    def driveToObj(self, dist):
        if dist < 66:
            self.twist.linear.x = 0.5
            return False
        else:
            self.twist.linear.x = 0
        return True

    def checkOffCol(self, col):
        print(self.distance)

        if Follower.distance < 0.6:
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
        else:
            return

    def checkComplete(self):
        tests = [self.foundRed,
                 self.foundBlue,
                 self.foundGreen,
                 self.foundYellow]
        
        if all(tests):
            self.foundAll = True

if __name__ == "__main__":
    rospy.init_node('follower')
    follower = Follower()
    rospy.spin()
