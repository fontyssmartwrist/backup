#!/usr/bin/env python

import socket
import rospy
import sys
import random
from math import pi

class ImitateCodesysValues():
    
    def __init__(self):

        HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        PORT = 65432       # Port to listen on (non-privileged ports are > 1023)

        # Creat a node
        rospy.init_node('echo_client', anonymous=True)
        
        
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        self.server_address = (HOST, PORT)
        rospy.loginfo('connecting to %s port %s', self.server_address[0], self.server_address[1])
        # Prevent node from shutting down
        self.send_to_ros()

    def send_to_ros(self):    
        self.sock.connect(self.server_address)
        self.sock.sendall("Send")
        try:
            while not rospy.is_shutdown():
                data = self.sock.recv(1024)
                extra = data.find(':') + 1
                if data[0] == 'n':
                    waypoint = str(int(data[3:len(data)]) - 1)
                    print(waypoint)
                else:
                    self.sock.sendall(data[extra:])
                    print(data)
                    if data[1:len(waypoint)+1] == waypoint:
                        rospy.sleep(0.5)
                        self.sock.sendall("Done")
                        print("Position reached succesfully")
                if not data:
                    break

        finally:
            # Close the socket
            rospy.loginfo('closing socket')
            self.sock.close()


if __name__ == '__main__':
    try:
        # Start the main class
        ImitateCodesysValues()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    