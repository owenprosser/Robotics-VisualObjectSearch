#!/usr/bin/env python
import rospy, cv2, cv_bridge, time, random
import numpy as np
from sensor_msgs.msg import Image, LaserScan, PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped


class Follower:
    distance = 10 #Distance: assigned using the laser scanner
    exploring = True # Bool used to tell when the robot is looking for an object
    seeColour = False # Bool used to tell when the robot can see a new object
    direction = 0 # [0,1] used to tell the robot to turn left or right
    count = 0 # Used to affect when the turn direction is changed

    foundRed = False    #{
    foundGreen = False  #
    foundBlue = False   #   Bools used to tell if an object has been seen before and then if they have all been found
    foundYellow = False #
    foundAll = False    #{

    redBound = (np.array([ 0, 50, 50]), np.array([0, 255, 255]))        #  
    greenBound = (np.array([ 50, 150, 50]), np.array([100, 255, 255]))  # The BGR boundaries for the colours
    blueBound = (np.array([ 100, 150, 50]), np.array([150, 255, 255]))  #       of the target objects
    yellowBound = (np.array([ 30, 200, 100]), np.array([50, 255, 195])) #

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #Setup Subscibers
        # -- Image data subscriber from the camera
        self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
        # -- Laser scan subscriber
        self.laser_scan = rospy.Subscriber('/turtlebot/scan',LaserScan,self.laser_callback)
        
        #Setup Publishers
        # -- Publish to make the robot move
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.twist = Twist()

    def image_callback(self, msg):
        self.seeColour = False
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w, d = image.shape

        redMask = cv2.inRange(hsv, self.redBound[0], self.redBound[1])          # Any colour seen by the camera which
        greenMask = cv2.inRange(hsv, self.greenBound[0], self.greenBound[1])    # fits into the defined area will show
        blueMask = cv2.inRange(hsv, self.blueBound[0], self.blueBound[1])       # as white in these images
        yellowMask = cv2.inRange(hsv, self.yellowBound[0], self.yellowBound[1]) #

        redMoment = cv2.moments(redMask)                                        # Moment of the masks from above.
        greenMoment = cv2.moments(greenMask)                                    # Used to steer robot towards objects
        blueMoment = cv2.moments(blueMask)                                      #  
        yellowMoment = cv2.moments(yellowMask)                                  #

        allMasks = redMask + blueMask + yellowMask + greenMask                  # Combination of all masks used to display

        cv2.imshow("all masks", allMasks) # Displays all masks
        self.checkComplete() # checks search task has not been completed

        if self.count == 200: # Function used to change the direction the robot will turn when exploring
            self.direction = random.randint(0,1)
            self.count = 0
        elif self.count < 200:
            self.count += 1 

        if self.foundAll == False: # checks that there are still objects to be found
            if self.foundRed == False and cv2.countNonZero(redMask) > 0: # If any of an object not yet found can be seen
                self.exploring = False                          # Stop exploring
                self.seeColour = True                           # Declare it can see a new colour
                cx = int(redMoment['m10']/redMoment['m00'])     # find which direction the robot needs to turn (x)
                cy = int(redMoment['m01']/redMoment['m00'])     # Same as above for y
                err = cx - w/2                                  # find the needed twist to face object
                self.twist.angular.z = -float(err) / 100        # Moves the robot to face the object
                dist = (redMoment['m10'] / 100000000)           # Finds the distance to object using its size to the camera
                if self.driveToObj(dist) is True:               # Drives towards object
                    self.checkOffCol(0)                         # Once at object it is checked off so wont be found again
            if self.foundGreen == False and cv2.countNonZero(greenMask) > 0: #The same as first IF statement
                self.exploring = False
                self.seeColour = True
                cx = int(greenMoment['m10']/greenMoment['m00'])
                cy = int(greenMoment['m01']/greenMoment['m00'])
                err = cx - w/2
                self.twist.angular.z = -float(err) / 100
                dist = (greenMoment['m10'] / 100000000)
                if self.driveToObj(dist) is True:
                    self.checkOffCol(1)
            if self.foundBlue == False and cv2.countNonZero(blueMask) > 0: #The same as first IF statement
                self.exploring = False
                self.seeColour = True
                cx = int(blueMoment['m10']/blueMoment['m00'])
                cy = int(blueMoment['m01']/blueMoment['m00'])
                err = cx - w/2
                self.twist.angular.z = -float(err) / 100
                dist = (blueMoment['m10'] / 100000000)
                if self.driveToObj(dist) is True:
                    self.checkOffCol(2)
            if self.foundYellow == False and cv2.countNonZero(yellowMask) > 0: #The same as first IF statement
                self.exploring = False
                self.seeColour = True
                cx = int(yellowMoment['m10']/yellowMoment['m00'])
                cy = int(yellowMoment['m01']/yellowMoment['m00'])
                err = cx - w/2
                self.twist.angular.z = -float(err) / 100
                dist = (yellowMoment['m10'] / 100000000)
                if self.driveToObj(dist) is True:
                    self.checkOffCol(3)
            if self.seeColour == False:
                self.search()
        else: # If all objects have been found.
            print("Found all Coloured Poles")

        self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("Original Image",image)
        cv2.waitKey(3)

    def search(self):
        if np.isnan(Follower.distance): # If the distance from the scanner is NaN type 
            self.rotate()               # Rotates the robot away from wall
        else:
            if Follower.distance < 1:       # If the robot is close to a wall
                self.twist.linear.x = 0     # Stop moving forward
                self.twist.angular.z = 0    # Stop roating from previous command (if there was one)
                self.rotate()               # Calls rotate() to move robot
            else:                           # -- If robot is not close to wall
                self.twist.linear.x = 0.5   # Move foarward
                self.twist.angular.z = 0    # Stop rotating to move in straight line

        self.cmd_vel_pub.publish(self.twist)
        
    def rotate(self):
        self.twist.linear.x = 0             # Stops moving forwards. To not hit wall
        if Follower.distance < 2:           # Checks that it is not in open space
            if self.direction == 0:         # If it is going to turn Left
                self.twist.angular.z = 1    # Publish positive number to turn left
            else:                           # -- If it is going to turn right
                self.twist.angular.z = -1   # Publish negative number to turn right
        else:                               # --  If it is in open space
            self.twist.angular.z = 0        # Stop rotating
        if np.isnan(Follower.distance):     # -- If the distance is type NaN 
            self.twist.angular.z = 0.5      # Rotate left slowly

        self.cmd_vel_pub.publish(self.twist) # Send twist commands to the publisher

    def laser_callback(self, msg):
        Follower.distance = min(msg.ranges) # Gets and Sets the distance from Laser Scanner
        #Follower.distance = msg.ranges[len(msg.ranges)/2]

    def driveToObj(self, dist):
        if dist < 66:                   # -- If the object is too far away based on moment in Image_Callback()
            self.twist.linear.x = 0.7   # Move robot forward    
            return False                # Return FALSE as robot has not yet reached object
        else:                           # -- If the robot is close enough to the object
            self.twist.linear.x = 0     # Stop moving forwards
        return True                     # Return TRUE as robot has reached the object

    def checkOffCol(self, col):
        if Follower.distance < 0.6:     # -- Checks that the robot is close to an object
            if col == 0:                # -- Looks at the parmeter pass from Image_Callback()
                self.foundRed = True    # Checks off colour
                print("Red Complete")   # Prints which colour has been found
                self.exploring = True   # Starts exploring again
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
            else:                       # -- If it has been passed a number out of range: break
                return
        else:                           # -- If the robot is not infront of an object: break
            return

    def checkComplete(self):
        tests = [self.foundRed,     #
                 self.foundBlue,    # Combines all bools into 1 array
                 self.foundGreen,   #
                 self.foundYellow]  #
        
        if all(tests):              # -- If all of the bools are TRUE
            self.foundAll = True    # Sets foundAll to TRUE

if __name__ == "__main__":
    rospy.init_node('follower')
    follower = Follower()
    rospy.spin()
