#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
# from move_group_pkg.srv import SetPub, SetPubRequest, SetPubResponse 

class TranslateData():
    def __init__(self):
        rospy.init_node('talker', anonymous=True)
        self.flag = False
        self.set_pub_service = rospy.Service('set_pub', Empty, self.set_pub_callback)
        self.pub = rospy.Publisher('/points_data',JointState, queue_size=20) # use to publish the joints states that need to be collected during performing tasks
        # self.sub = rospy.Subscriber("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, self.sub_callback)
        self.sub = rospy.Subscriber("/joint_states", JointState, self.sub_callback)
    
    def sub_callback(self, data):
        """
        use a flag to indicate when to publish the data that needed to be collected
        """
        if self.flag:
            self.pub.publish(data)
    
        
    def set_pub_callback(self, data):
        """
        callback function for set publisher service
        """        
        res = EmptyResponse()
        self.flag = not self.flag
        # res = self.flag
        return res
    

    
if __name__ == '__main__':
    
    translate_data = TranslateData()
    
    rospy.spin()