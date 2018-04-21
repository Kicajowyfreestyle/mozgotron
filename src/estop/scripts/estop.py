#!/usr/bin/env python                                                           
                                                                                
import rospy                                                                       
from std_msgs.msg import Bool                                                      
from sensor_msgs.msg import Range                                                  
                                                                                   
class Estop:                                                                       
    def __init__(self):                                                            
        self.pub = None                                                            
        self.proximity = [False]*8                                                 
                                                                                   
    def initPublisher(self):                                                       
        self.pub = rospy.Publisher('estop', Bool, queue_size=10)                   
                                                                                   
    def calculate(self, dist):                                                     
        frame_id = int(dist.header.frame_id[-1])
        if dist.range < 0.03:
            self.proximity[frame_id] = True
            self.pub.publish(True)                                                     
                                                                                   
estop = Estop()                                                                    
                                                                                   
def callback(vel):                                                                 
    estop.calculate(vel)                                           
                                                                                   
if __name__ == '__main__':                                                         
    rospy.init_node('estop', anonymous=True)                                       
    estop.initPublisher()                                                          
                                                                                   
    rospy.Subscriber('proximity0', Range, callback)                                
    rospy.Subscriber('proximity1', Range, callback)                                
    rospy.Subscriber('proximity2', Range, callback)                                
    rospy.Subscriber('proximity3', Range, callback)                                
    rospy.Subscriber('proximity4', Range, callback)                                
    rospy.Subscriber('proximity5', Range, callback)                                
    rospy.Subscriber('proximity6', Range, callback)                                
    rospy.Subscriber('proximity7', Range, callback)                                
                                                                                   
    rospy.spin()  
