#!/usr/bin/env python

import rospy
import socket
import threading
import time
from thread import *
from sensor_msgs.msg import JointState
from smart_movement.msg import codesys_joint 
from std_msgs.msg import String
from math import pi, ceil

class ServerThread(threading.Thread):

    def __init__(self, conn):      
        self.connection = conn
        data = conn.recv(1024)
        rospy.loginfo(data)
        if data[0:5] == "write":
            rospy.loginfo("Sender connected")
            # Subsribe to message sender
            listen = rospy.Subscriber('/codesys_waypoints', codesys_joint, self.callback)
        elif data[0:4] == "read":
            rospy.loginfo("Reader connected")
        else:
            rospy.logwarn("Undefined client")

        while not rospy.is_shutdown():
            data = conn.recv(1024)
            rospy.loginfo(data)
            if not data:
                break
            elif data[0:4] == "Done":
                rospy.loginfo("Position reached succesfully")
                status.publish("Done")                 
            elif data[0:4] == "enc:":
                self.string_to_message(data)
            else:
                rospy.logwarn("Unable to use that data")

    def callback(self, message):
        # Send header
        scale = 2
        waypoint = 'ns:(' + str(int((message.waypoints-1)/scale)) + ')'
        self.connection.sendall(waypoint)
        rospy.loginfo(waypoint)
        rospy.loginfo(message.waypoint)
        counter = 0
        # Loop through sending all the waypoints
        for wp_nr in range(1, message.waypoints-1, scale):
            # Makes sure the last waypoint send is the same as the last calculated waypoint
            if wp_nr + scale > message.waypoints-1:
                wp_nr = message.waypoints-1
            # Create the message
            waypoint = 'w' + str(counter)+ ':[' 
            waypoint += str(message.theta0[wp_nr]) + ',' + str(message.theta1[wp_nr]) + ','
            waypoint += str(message.theta2[wp_nr]) + ',' + str(message.theta3[wp_nr]) + ']'
            rospy.loginfo(waypoint)
            # Send the message
            self.conn.sendall(waypoint)
            counter += 1
            # For simulation this wait function is used
            rospy.sleep(0.05)


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
        real_angles.publish(current_pos)

def main():
    # '127.0.0.1' '169.254.141.118' 
    HOST = '169.254.141.118'         # Standard loopback interface address (localhost)
    PORT = 65432                # Port to listen on (non-privileged ports are > 1023)
    # Creat a node
    rospy.init_node('lord_server')
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Bind the socket to the port
    server_address = (HOST, PORT)
    rospy.loginfo('Starting up multithreaded server on %s port %s', server_address[0], server_address[1])
    sock.bind(server_address)
    # Listen for incoming connections
    sock.listen(3)
    global status 
    status = rospy.Publisher('/goal_reached', String, queue_size=10)
    # Publisher for send real angels
    global real_angles
    real_angles = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size= 10)
    
    try:
        while not rospy.is_shutdown():
            conn, addr = sock.accept()
            rospy.loginfo('Connection from %s', addr)
            start_new_thread(ServerThread, (conn,))
 
    except rospy.ROSException:
        sock.close() 

    finally:
        rospy.loginfo('Closing socket')
        sock.close()
        time.sleep(0.5)


if __name__ == '__main__':
    try:
        # Start the main class
        main()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    