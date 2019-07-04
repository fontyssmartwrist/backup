#!/usr/bin/env python

import rospy
import socket
import sys
from sensor_msgs.msg import JointState
from smart_movement.msg import codesys_joint 
from std_msgs.msg import String
from math import pi

class GiveAnglesToROS():

    def __init__(self):
        # '127.0.0.1'
        HOST = '169.254.141.118'    # Standard loopback interface address (localhost)
        PORT = 65432                # Port to listen on (non-privileged ports are > 1023)

        # Creat a node
        rospy.init_node('lord_server', anonymous=True)
        self.real_angles = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size= 10)
        # Tell the commander to send the new goal
        self.status = rospy.Publisher('/goal_reached', String, queue_size=10)
        # Create a TCP/IP socket
        listen = rospy.Subscriber('/codesys_waypoints', codesys_joint, self.callback)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind the socket to the port
        self.server_address = (HOST, PORT)
        rospy.loginfo('Starting up on %s port %s', self.server_address[0], self.server_address[1])
        self.sock.bind(self.server_address)
        # Listen for incoming connections
        self.sock.listen(1)
        self.server()

    def server(self):
        try:

            while not rospy.is_shutdown():
                # Wait for a connection
                self.connected = False
                rospy.loginfo('Waiting for a connection')
                conn, addr = self.sock.accept()
                # Making the conn variable globaly availible 
                self.conn = conn
                rospy.loginfo('Connection from %s', addr)
                # Enable the sendback to send messages to codesys
                self.connected = True

                while not rospy.is_shutdown():
                    # Send data
                    #conn.settimeout(5.0)
                    data = conn.recv(1024)
                    rospy.loginfo(data)
                    if not data:
                        break
                    elif data[0:4] == "Done":
                        rospy.loginfo("Position reached succesfully")
                        self.status.publish("Done")                 
                    else:
                        self.string_to_message(data)

                    
        finally:
            # Clean up the connection
            rospy.loginfo('Closing socket')
            self.conn.close()
            conn.close()
            
            


    def callback(self, message):
        #rospy.loginfo(message)
        if self.connected is True:
            # Send header
            waypoint = 'ns:(' + str(message.waypoints-1) + ')'
            self.conn.sendall(waypoint)
            rospy.loginfo(waypoint)
            # Loop through sending all the waypoints
            for wp_nr in range(1, message.waypoints):
                    # Create the message
                    waypoint = 'w' + str(wp_nr-1)+ ':[' 
                    waypoint += str(message.theta0[wp_nr]) + ',' + str(message.theta1[wp_nr]) + ','
                    waypoint += str(message.theta2[wp_nr]) + ',' + str(message.theta3[wp_nr]) + ']'
                    rospy.loginfo(waypoint)
                    # Send the message
                    self.conn.sendall(waypoint)
                    # For simulation this wait function is used
                    rospy.sleep(0.05)

                    
        # If there was no connection the following warning will be displayed
        else:
            rospy.logwarn("Connection not established")

    def string_to_message(self, res_msg):
        first = res_msg.find('[')
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
                                    int(res_msg[first+1:comma1]) * pi / 3140,
                                    int(res_msg[comma1+1:comma2]) * pi / 3140,
                                    int(res_msg[comma2+1:comma3]) * pi / 3140,
                                    int(res_msg[comma3+1:end]) * pi / 3140]
        #rospy.loginfo(current_pos)
        self.real_angles.publish(current_pos)

if __name__ == '__main__':
    try:
        # Start the main class
        GiveAnglesToROS()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    