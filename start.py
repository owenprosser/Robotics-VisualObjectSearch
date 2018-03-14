 #!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge

class Follower:
  distance = 0
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    #Setup Subscibers
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.laser_scan = rospy.Subscriber('/turtlebot/scan',LaserScan,self.laser_callback)

    self.map_sub = rospy.Subscriber("/turtlebot/move_base/global_costmap/costmap", OccupancyGrid, self.map_callback)
    
    #Setup Publishers
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)

    self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=10)
    
    self.twist = Twist()

  def map_callback(self, map_data):
    #np.savetxt('map_data', map_data.data)
    self.origin_x = map_data.info.origin.position.x
    self.origin_y = map_data.info.origin.position.y
    self.origin_z = map_data.info.origin.position.z

    now = rospy.get_rostime()
    
    mv_goal = PoseStamped()
    mv_goal.header.stamp = now
    mv_goal.header.frame_id = 'map'
    mv_goal.pose = Pose()
    mv_goal.pose.position = Point()
    mv_goal.pose.position.x = 0.0
    mv_goal.pose.position.y = 0.0
    mv_goal.pose.position.z = 0.0
    mv_goal.pose.orientation = Quaternion()
    mv_goal.pose.orientation.w = 1.0

    self.goal_pub.publish(mv_goal)
    
  def image_callback(self, msg):
    if (follower.distance > 2):
        self.twist.linear.x = 0.0
    else:
        self.twist.linear.x = 0.0

    self.cmd_vel_pub.publish(self.twist)

  def laser_callback(self, msg):
    Follower.distance = msg.ranges[10]
    Follower.distance = numpy.nanmean(msg.ranges)
    print(str(Follower.distance))

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL