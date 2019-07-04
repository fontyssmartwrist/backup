#!/usr/bin/env python

import rospy
import socket
import sys

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

# Creat a node
rospy.init_node('echo_server', anonymous=True)
# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
server_address = (HOST, PORT)
rospy.loginfo('starting up on %s port %s', server_address[0], server_address[1])
sock.bind(server_address)
# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection
    rospy.loginfo('waiting for a connection')
    conn, addr = sock.accept()

    try:
        rospy.loginfo('connection from %s', client_address)

        while True:
            
            data = conn.recv(1024)
            if not data:
                break
            rospy.loginfo('received "%s"', data)
            
    finally:
        # Clean up the connection
        conn.close()
