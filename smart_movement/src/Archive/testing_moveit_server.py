#!/usr/bin/env python

import rospy
import socket
import sys
from sensor_msgs.msg import JointState
from math import pi

class GiveAnglesToROS():

    def __init__(self):
        HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

        # Creat a node
        rospy.init_node('echo_server', anonymous=True)
        self.real_angles = rospy.Publisher('/current_joint_values', JointState, queue_size= 10)
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind the socket to the port
        self.server_address = (HOST, PORT)
        rospy.loginfo('starting up on %s port %s', self.server_address[0], self.server_address[1])
        self.sock.bind(self.server_address)
        # Listen for incoming connections
        self.sock.listen(1)
        self.server()

    def server(self):
        try:

            while not rospy.is_shutdown():
                # Wait for a connection
                rospy.loginfo('waiting for a connection')
                conn, addr = self.sock.accept()
                rospy.loginfo('connection from %s', addr)

                full_message = ''

                while not rospy.is_shutdown():
                    # Send data
                    data = conn.recv(1024)
                    if not data:
                        break
                    full_message += data
                    if ']' in data:
                        rospy.loginfo(full_message)
                        self.string_to_message(full_message)
                        full_message = ''



        finally:
            # Clean up the connection
            rospy.loginfo('closing socket')
            conn.close()

    def string_to_message(self, res_msg):
        comma1 = res_msg.find(',')
        comma2 = res_msg.find(',',comma1+1)
        comma3 = res_msg.find(',',comma2+1)
        end = res_msg.find(']')
        current_pos = JointState()
        current_pos.header.seq = 0
        current_pos.header.stamp = rospy.Time.now()
        current_pos.header.frame_id = "/world"
        current_pos.name = ["end_link", "joint0", "joint1", "joint3", "joint4"]
        current_pos.position = [0.0, 
                                    int(res_msg[1:comma1]) * pi / 100000,
                                    int(res_msg[comma1+1:comma2]) * pi / 100000,
                                    int(res_msg[comma2+1:comma3]) * pi / 100000,
                                    int(res_msg[comma3+1:end]) * pi / 100000]
        rospy.loginfo(current_pos)
        self.real_angles.publish(current_pos)




if __name__ == '__main__':
    try:
        # Start the main class
        GiveAnglesToROS()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    