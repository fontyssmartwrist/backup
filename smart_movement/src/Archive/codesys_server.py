#!/usr/bin/env python

import socket
import sys
import rospy

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65430        # Port to listen on (non-privileged ports are > 1023)

# Creat a node
rospy.init_node('echo_client', anonymous=True)
# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect the socket to the port where the server is listening
server_address = (HOST, PORT)
rospy.loginfo('connecting to %s port %s', server_address[0], server_address[1])
sock.bind(server_address)
sock.listen(1)

try:
    while not rospy.is_shutdown():
    
        rospy.loginfo('waiting for a connection')
        conn, addr = sock.accept()
        rospy.loginfo('connection from %s', addr)
        full_message = ''
    
        while not rospy.is_shutdown():
            # Send data
            data = conn.recv(1024)
            if not data:
                break
            rospy.loginfo(data)


finally:
    rospy.loginfo('closing socket')
    sock.close()