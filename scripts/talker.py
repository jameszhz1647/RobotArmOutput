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
        self.pub = rospy.Publisher('joints_data', JointState, queue_size=10)
        self.sub = rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, self.sub_callback)
    
            
    def sub_callback(self, data):
        if self.flag:
            self.pub.publish(data)
        
    def set_pub_callback(self, data):
        res = EmptyResponse()
        self.flag = not self.flag
        # res = self.flag
        return res
    

    
if __name__ == '__main__':
    
    translate_data = TranslateData()
    
    rospy.spin()