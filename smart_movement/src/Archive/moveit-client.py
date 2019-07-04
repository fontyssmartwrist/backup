#!/usr/bin/env python

import socket
import rospy
import sys
from smart_movement.msg import codesys_joint

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
        rospy.loginfo('connecting to %s port %s', self.server_address[0], self.server_address[1])
        # Listen to the topic that sends waypoints
        listen = rospy.Subscriber('/codesys_waypoints', codesys_joint, self.callback)
        # Prevent node from shutting down
        self.send_to_codesys()

    def send_to_codesys(self):    
        self.sock.connect(self.server_address)
        self.incoming_data = 1 # Change this back
        
        
        try:
            while not rospy.is_shutdown():
                theta0 = 't0:['
                theta1 = 't1:['
                theta2 = 't2:['
                theta3 = 't3:['
                msg_part = 0
                find_comma = 250
                stop = False

                

                if self.incoming_data is not 0:
                    """
                    # Put message in data
                    message = self.incoming_data
                    length = 'Amount of waypoints is ' + str(message.waypoints) + '\n'
                    for amount in range(0, message.waypoints):
                        theta0 += str(message.theta0[amount]) + ','
                        theta1 += str(message.theta0[amount]) + ','
                        theta2 += str(message.theta0[amount]) + ','
                        theta3 += str(message.theta0[amount]) + ','
                    
                    # Send data
                    full_message = length + theta0[:-1] + ']' + theta1[:-1] + ']' 
                    full_message += theta2[:-1] + ']' + theta3[:-1] + ']|'
                    """
                    full_message[0] = 't0:[0,1563,3126,4689,6253,7816,9379,10943,12506,14069,15633,17196,18759,20322,21886,23449,25012,26576,28139,29702,31266,32829,34392,35955,37519,39082,40645,42209,43772,45335,46899,48462,50025]'
                    full_message[1] = 't1:[0,1563,3126,4689,6253,7816,9379,10943,12506,14069,15633,17196,18759,20322,21886,23449,25012,26576,28139,29702,31266,32829,34392,35955,37519,39082,40645,42209,43772,45335,46899,48462,50025]'
                    full_message[2] = 't2:[0,1563,3126,4689,6253,7816,9379,10943,12506,14069,15633,17196,18759,20322,21886,23449,25012,26576,28139,29702,31266,32829,34392,35955,37519,39082,40645,42209,43772,45335,46899,48462,50025]'
                    full_message[3] = 't3:[0,1563,3126,4689,6253,7816,9379,10943,12506,14069,15633,17196,18759,20322,21886,23449,25012,26576,28139,29702,31266,32829,34392,35955,37519,39082,40645,42209,43772,45335,46899,48462,50025]|'

                    # Send max 250 charter long strings
                    while not rospy.is_shutdown() and stop is False:
                        if len(full_message) < msg_part + find_comma:
                            find_comma = len(full_message) - msg_part -1
                            stop = True
                        
                        if full_message[msg_part + find_comma] is ',' or full_message[msg_part + find_comma] is'|':
                            self.sock.sendall(full_message[msg_part:msg_part + find_comma+1])
                            rospy.loginfo(full_message[msg_part:msg_part + find_comma+1])
                            print(msg_part)
                            print(find_comma)
                            msg_part += find_comma + 1
                            find_comma = 250    
                        else:
                            find_comma -= 1
                        
                        
                    # Send full message        
                    
                    #self.incoming_data = 0 Change this back

                rospy.sleep(5)


        finally:
            # Close the socket
            rospy.loginfo('closing socket')
            self.sock.close()

    def callback(self, data):
        print('data received')
        self.incoming_data = data

if __name__ == '__main__':
    try:
        # Start the main class
        SendWayPointsToCodesys()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    