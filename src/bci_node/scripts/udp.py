#!/usr/bin/env python                                                           
import socket
import rospy                                                                       
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import json

class UDP():
    UDP_PORT = 8080

    def __init__(self):
        self.pub = None
        self.sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self.sock.bind(('', self.UDP_PORT))
        
        self.prev_data = {"x": 0.0, "y": 0.0}

    def initPublisher(self):
        self.pub = rospy.Publisher('set_destination', Pose2D, queue_size=10)

    def publish(self):
        data, addr = self.sock.recvfrom(1024)
        print "received message:", data
        data = json.loads(data)
        if data["cmd"] == "go":
            dx = data["x"] - self.prev_data["x"]
            dy = data["y"] - self.prev_data["y"]
            mov = dx + dy
            self.pub.publish(Pose2D(data["x"], data["y"], data["theta"]))
            rospy.sleep(8*mov + 2)

        self.prev_data = data
        

udp = UDP()

if __name__ == '__main__':
    rospy.init_node('udp', anonymous=True)                                       
    udp.initPublisher()

    while True:
        udp.publish()
