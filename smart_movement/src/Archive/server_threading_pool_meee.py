#!/usr/bin/python
import rospy
import Queue 
import socket 
import struct
import binascii
import sys
from threading import Thread 
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from smart_movement.msg import codesys_joint 
from std_msgs.msg import String
from math import pi
import time 

# Child class to create the instance of a thread_client and assign a task to it 

class ClientThread(Thread):

	def run(self):

		while True:
			print 'inside the thread'
			#check if the client data is stored in queue
			self.client = client_queue.get()

			if self.client is not None:
				print 'Connection made', self.client[1]
				self.recieved = self.client[0].recv(1024)
				
				# The client wants to recieve the waypoints recieved from the codesys_waypoints 
				if self.recieved[0:4] == "read":
					
					rospy.loginfo("Recieved read")
					self.done_pub = rospy.Publisher('/goal_reached', String, queue_size=10)
					while True:
						self.send_motor_waypoints()

				# The clients wants to send the enocder data to the fake_joint_controller 	
						
				if self.recieved[0:5] == "write":
					
					rospy.loginfo("Recieved write")
					while True:
						self.encoder_recieved = self.client[0].recv(1024)
						self.recieved_encoder_motor()				

				print 'my name is ',self.getName()
				print 'Communication Closed', self.client[1]
		
	def send_motor_waypoints(self):
		# #Check if done signal has being recieved from CodeSys to update the next position 
		# if self.recieved[0:4] == "Done":
		# 	connection = self.done_pub.get_num_connections()
		# 	if connection > 0 : 
		# 		self.done_pub.publish("update")
		# 		rospy.loginfo("The postion updated")
			
		# else: 
		# 	rospy.loginfo("readthe waypoint")
		rospy.loginfo("inside the function")
		# recieved = self.client[0].recv(1024)
		rospy.loginfo("recieved")
		flag = "True"

		while (flag == "True"):
			rospy.loginfo("read the waypoints")
			self.sub = rospy.Subscriber('/codesys_waypoints', codesys_joint, self.callback)
			message = self.client[0].recv(1024)
			while (message is None):
				pass 
			if (message[0:4] == "Done"):
				while (self.done_pub.get_num_connections() == 0):
					pass
				self.done_pub.publish("update")
		        rospy.loginfo("updated")

			#elif not message:
			#	break

		



		

		

	
	def callback(self, message):
		rospy.loginfo("inside the callback")
		scale = 2
		waypoint = 'ns:(' + str(int((message.waypoints-1)/scale)) + ')'
		self.client[0].sendall(waypoint)
		rospy.loginfo(message.waypoints)
		counter = 0
		for wp_nr in range(1, message.waypoints-1, scale):
			if wp_nr + scale > message.waypoints-1:
				wp_nr = message.waypoints-1
			waypoint = 'w' + str(counter)+ ':[' 
			waypoint += str(message.theta0[wp_nr]) + ',' + str(message.theta1[wp_nr]) + ','
			waypoint += str(message.theta2[wp_nr]) + ',' + str(message.theta3[wp_nr]) + ']'
			rospy.loginfo(waypoint)
			self.client[0].sendall(waypoint)
			counter += 1
			rospy.sleep(0.05)

        
		

	def recieved_encoder_motor(self):
		pub_encoder = rospy.Publisher('encoder_data', String, queue_size=10)
		while not rospy.is_shutdown(): 
			pub_encoder.publish(self.encoder_recieved)





# Define Server socket 
HOST = '169.254.141.118'
PORT = 65430
# Create a queu to store the client data 
client_queue = Queue.Queue(0)

#Create three threads 
for x in range (3): 
	ClientThread().start()

# Create the server object 
server_thread = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_thread.bind((HOST, PORT))
server_thread.listen(5)

if __name__ == '__main__' : 
	#create a node 
	rospy.init_node('server_thread_test', anonymous=True)

	while True:

		try: 
			client_queue.put(server_thread.accept())
		except rospy.ROSException:
			server_thread.close()




   