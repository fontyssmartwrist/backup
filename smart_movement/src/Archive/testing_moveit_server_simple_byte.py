#!/usr/bin/env python

import rospy
import socket
import sys
import struct
import binascii
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from math import pi

class GiveAnglesToROS():

    def __init__(self):
        HOST = '169.254.141.118'  # Standard loopback interface address (localhost)
        PORT = 65431        # Port to listen on (non-privileged ports are > 1023)

        # Creat a node
        rospy.init_node('echo_server', anonymous=True)
        self.real_angles = rospy.Publisher('/current_joint_values', JointState, queue_size= 10)
        # Create a TCP/IP socket
        self.receiving = rospy.Subscriber('/WOW', String, self.callback )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind the socket to the port
        self.server_address = (HOST, PORT)
        rospy.loginfo('starting up on %s port %s', self.server_address[0], self.server_address[1])
        self.sock.bind(self.server_address)
        # Listen for incoming connections
        self.sock.listen(1)
        self.connected = False
        self.server()

    def server(self):
        try:

            while not rospy.is_shutdown():

                # Wait for a connection
                rospy.loginfo('waiting for a connection')
                conn, addr = self.sock.accept()
                rospy.loginfo('connection from %s', addr)
                value = 5000
                self.connected = True
                #self.conn.sendall(str(value))
                #rospy.loginfo("Sending the value: %s", str(value))
                # full_message = ''
                value += 5000
                stuff = [9, 10, 11, 17]
                self.key = bytearray(stuff)
                while not rospy.is_shutdown():
                    conn.send(self.key)
                    rospy.loginfo(self.key)
                    rospy.sleep(0.5)

                while not rospy.is_shutdown():
                    # Send data
                    data = conn.recv(1024)
                    rospy.loginfo(data)
                    #self.conn.sendall(str(value))
                    #rospy.loginfo("Sending the value: |%s|", str(value))
                    #value += 5000
                    if not data:
                        break
                    

        finally:
            # Clean up the connection
            rospy.loginfo('closing socket')
            conn.close()
            self.connected = False


    def callback(self, message):
        theta=['','','','']
        theta[0] = 't01:[61421, 57654, 53887, 50120, 46353, 42587, 38820, 35053, 31286, 27519, 23753, 19986, 16219, 12452, 8686, 4919, 1152]'
        theta[1] = 't11:[14976, 17165, 19354, 21543, 23732, 25921, 28110, 30299, 32488, 34678, 36867, 39056, 41245, 43434, 45623, 47812, 50001]'
        theta[2] = 't21:[36322, 33977, 31632, 29287, 26942, 24597, 22252, 19907, 17562, 15217, 12872, 10527, 8182, 5838, 3493, 1148, 1196]'
        theta[3] = 't31:[41981, 42487, 42993, 43499, 44005, 44511, 45017, 45523, 46029, 46535, 47041, 47547, 48053, 48558, 49064, 49570, 50076]'
        if self.connected is True:
            """
            rospy.loginfo("Sending the following value: |%s|", theta[0])
            self.conn.sendall(theta[0])
            rospy.loginfo("Sending the following value: |%s|", theta[1])
            self.conn.sendall(theta[1])
            rospy.loginfo("Sending the following value: |%s|", theta[2])
            self.conn.sendall(theta[2])
            rospy.loginfo("Sending the following value: |%s|", theta[3])
            self.conn.sendall(theta[3])
            """

        else:
            rospy.logerr("Cannot send: %s", message.data)

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