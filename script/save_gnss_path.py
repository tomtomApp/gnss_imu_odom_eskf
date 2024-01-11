#! /usr/bin/env python3
import math
import pandas as pd

# import for ros function
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Save_GNSS_Path():
    def __init__(self):
        rospy.init_node("save_gnss_path", anonymous=True)
        self.savepath_sub = rospy.Subscriber("/gnss_pose", PoseStamped, self.gnss_callback)
        print('gnss Data:',self.savepath_sub)
        # pre odometry
        self.pre_x = 0.0 #[m]
        self.pre_y = 0.0 #[m]
        self.dist_thread = 0.01 #[m]

        # path dict for csvfile
        self.path_dict = {}
    
    def gnss_callback(self, gnss):
        dist = math.sqrt( (gnss.pose.position.x - self.pre_x)**2 + (gnss.pose.position.y - self.pre_y)**2 )
        if dist >= self.dist_thread:
            current_point = [gnss.pose.position.x,
                             gnss.pose.position.y,
                             gnss.pose.position.z,
                             gnss.pose.orientation.x,
                             gnss.pose.orientation.y,
                             gnss.pose.orientation.z,
                             gnss.pose.orientation.w,
                            ]
            self.path_dict[len(self.path_dict)] = current_point

            # save pre_position
            self.pre_x = gnss.pose.position.x
            self.pre_y = gnss.pose.position.y
            print(current_point)
        else:
            pass
    """
    def odom_callback(self, odom):
        current_point = [odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z,
                         odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z,
                         odom.pose.pose.orientation.w,
                         ]
        self.path_dict[len(self.path_dict)] = current_point
    """
    def save_csv(self):
        # Save CSV path fileSave CSV path file
        print("Save CSV path file")
        cols = ["x", "y", "z", "w0", "w1", "w2", "w3"]
        df = pd.DataFrame.from_dict(self.path_dict, orient='index',columns=cols)
        df.to_csv("~/catkin_ws/src/gnss_imu_odom_eskf/gnss_path/gnss_log_test1.csv", index=False)

if __name__ == '__main__':
    print('Save Path is Started...')
    save_path = Save_GNSS_Path()

    while not rospy.is_shutdown():
        pass
    save_path.save_csv()
    print("Save Path finished!")
