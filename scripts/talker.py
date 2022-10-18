#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
# from move_group_pkg.srv import SetPub, SetPubRequest, SetPubResponse 
import moveit_msgs.msg
import trajectory_msgs.msg

class TranslateData():
    def __init__(self):
        rospy.init_node('talker', anonymous=True)
        self.flag = False
        self.set_pub_service = rospy.Service('set_pub', Empty, self.set_pub_callback)
        self.pub = rospy.Publisher('/points_data', trajectory_msgs.msg.JointTrajectoryPoint, queue_size=20)
        self.sub = rospy.Subscriber("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, self.sub_callback)
    
            
    def sub_callback(self, data):
        # joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        if self.flag:
            joint_trajectory = data.trajectory[0].joint_trajectory
            print(type(joint_trajectory))
            for point in joint_trajectory.points:
                self.pub.publish(point)
             
            # /move_group/display_planned_path/trajectory/joint_trajectory/points
        
    def set_pub_callback(self, data):
        res = EmptyResponse()
        self.flag = not self.flag
        # res = self.flag
        return res
    

    
if __name__ == '__main__':
    
    translate_data = TranslateData()
    
    rospy.spin()