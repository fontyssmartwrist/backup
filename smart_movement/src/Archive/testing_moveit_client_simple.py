#!/usr/bin/env python

import socket
import rospy
import sys
import math
#from smart_movement.msg import codesys_joint

class SendWayPointsToCodesys():
    def __init__(self):

        HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        PORT = 65430        # Port to listen on (non-privileged ports are > 1023)

        # Creat a node
        rospy.init_node('echo_client', anonymous=True)
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        self.server_address = (HOST, PORT)
        # Listen to the topic that sends waypoints
        #listen = rospy.Subscriber('/codesys_waypoints', codesys_joint, self.send_to_codesys)
        # Prevent node from shutting down
        self.send_to_codesys()

        
        

    def send_to_codesys(self):    
        rospy.loginfo('connecting to %s port %s', self.server_address[0], self.server_address[1])
        self.sock.connect(self.server_address)

        try:
                
            while not rospy.is_shutdown():

                # Setup and clear theta variable
                theta=['','','','']
                # Put message in data
                """
                for amount in range(0, message.waypoints):
                    for joint_nb in range(0,4):
                        theta[joint_nb] += str(message.theta0[amount]) 
                        if amount is message.waypoints -1:
                            theta[joint_nb] += ']'
                        else:
                            theta[joint_nb] += ','  
                """

                # Hardcoded data
                theta[0] = [549,516,483,450,417,384,351,318,285,252,219,186,153,120,87,54,21,-11,-44,-77,-110,-143,-176,-209,-242,-275,-308,-341,-374,-407,-440,-473,-506,-539,-572,-605,-638,-671,-704,-737,-770,-803,-836,-869,-902,-935,-968,-1001,-1034,-1067,-1100]
                theta[1] = [-11082,-13861,-16639,-19417,-22196,-24974,-27753,-30531,-33310,-36088,-38867,-41645,-44423,-47202,-49980,-52759,-55537,-58316,-61094,-63873,-66651,-69429,-72208,-74986,-77765,-80543,-83322,-86100,-88879,-91657,-94435,-97214,-99992,-102771,-105549,-108328,-111106,-113885,-116663,-119441,-122220,-124998,-127777,-130555,-133334,-136112,-138890,-141669,-144447,-147226,-150004]
                theta[2] = [36917,35135,33353,31571,29789,28008,26226,24444,22662,20880,19098,17316,15534,13753,11971,10189,8407,6625,4843,3061,1279,-501,-2283,-4065,-5847,-7629,-9411,-11193,-12975,-14756,-16538,-18320,-20102,-21884,-23666,-25448,-27230,-29011,-30793,-32575,-34357,-36139,-37921,-39703,-41485,-43266,-45048,-46830,-48612,-50394,-52176]
                theta[3] = [-60458,-58249,-56039,-53830,-51621,-49411,-47202,-44992,-42783,-40574,-38364,-36155,-33945,-31736,-29527,-27317,-25108,-22898,-20689,-18479,-16270,-14061,-11851,-9642,-7432,-5223,-3014,-804,1404,3614,5823,8032,10242,12451,14661,16870,19079,21289,23498,25708,27917,30126,32336,34545,36755,38964,41174,43383,45592,47802,50011]

                """
                # Send max 255 charter long strings
                for joint in range(0,4):
                    msg_part = 0
                    index = 1
                    while not rospy.is_shutdown():
                        # Create variable to find the last comma within the 250 char boundery
                        msg_end = 245
                        # Store into easier variable name
                        read = theta[joint][msg_part:msg_part + msg_end + 1]
                        # Make the start of the message
                        msg = 't'+str(joint)+str(index)+':['
                        # Add or remove parts of read to add the values to msg 
                        if not read.rfind(']') is -1:
                            msg_end = read.rfind(']')
                            if read.find(',') is 0:
                                msg = msg + read[1:msg_part + msg_end + 1]
                            else:
                                msg = msg + read[0:msg_part + msg_end + 1]
                        elif index is 1:
                            msg_end = read.rfind(',')
                            msg = msg + read[:msg_part + msg_end] + ']' 
                        else:
                            msg_end = read.rfind(',')
                            msg = msg + read[1:msg_part + msg_end] + ']' 
                        # Print the created message
                        rospy.loginfo(msg)
                        index += 1
                        msg_part = msg_end
                        # Send single message
                        self.sock.sendall(msg)
                        # If ] symbol is found in the message break
                        if not read.rfind(']') is -1:
                            break
                """
                
                for wp_nr in range(0, len(theta[0])):
                    waypoint = theta[wp_nr] + theta[wp_nr] + theta[wp_nr] + theta[wp_nr]
                    print(waypoint)



                # Change the rate of sending messages
                rospy.sleep(3)
        
        finally:
            # Close the socket
            rospy.loginfo('closing socket')
            self.sock.close()


       

if __name__ == '__main__':
    try:
        # Start the main class
        SendWayPointsToCodesys()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    