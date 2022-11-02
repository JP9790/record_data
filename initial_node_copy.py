#!/usr/bin/env python3

"""
	
Services:
	- /iiwa/iiwa_fk_server
"""
#ROS Imports
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension, Int16MultiArray
from iiwa_tools.srv import GetFK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


#Non-Ros Imports
import numpy as np
from math import pi
import csv


class Control:
	def __init__(self):
		
		#define time step
		self.DT = 1./500.
		#define robot state as joint state message
		self.rob_state = JointState()
		#define a variable for keeping track of time stamp of recorded data
		self.data_time = 0

	#define subscribers and publishers and timers
	
	#subscribers
		#subcribe to joint state topic to obtain joint angles, joint velocities and joint efforts
		self.joint_sub = rospy.Subscriber("/iiwa/joint_states", JointState, self.update_state, queue_size = 10)
	
	#publishers
		#publisher for demo data
		#self.motion = rospy.Publisher("demo_data", Float64MultiArray, queue_size = 5)
		self.record_data = rospy.Publisher("record_data", Float64MultiArray, queue_size = 5)
		


	#setup fk client
		self.fk_service = '/iiwa/iiwa_fk_server'
		rospy.wait_for_service(self.fk_service)
		self.get_fk = rospy.ServiceProxy(self.fk_service, GetFK)
		self.seed = Float64MultiArray()
		self.seed.layout = MultiArrayLayout()
		self.seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
		self.seed.layout.dim[0].size = 1
		self.seed.layout.dim[1].size = 7
		self.seed.data = self.rob_state.position
        
		
	
	
	#Initialize variable for storing the experiment data
		self.mydata = Float64MultiArray()
		self.mydata.layout = MultiArrayLayout()
		self.mydata.layout.dim = [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
		self.mydata.layout.dim[0].label = "Time"
		self.mydata.layout.dim[0].size = 1
		self.mydata.layout.dim[0].stride = 1
		self.mydata.layout.dim[1].label = "Desired Motion Data"
		self.mydata.layout.dim[1].size = 1
		self.mydata.layout.dim[1].stride = 3
		
		#other variables
		self.check_time = 0;
		self.start_time = 0;
		
#callback functions

	#callback function for robot joint state subscriber		
	def update_state(self,data):
		#update the state of the robot and store the position (joint angles), velocity (joint velocities), and effort (joint efforts)
		self.rob_state = data
		
	#function for obtaining the forward kinematics data from the forward kinematics server
	def collect_fk_data(self):
		#set forward kinematics serve data as the joint angles of the robot from joint state message
		self.seed.data = self.rob_state.position
		#call the forward kinematics server to compute the cartesian position and orientation of the robot in its base frame
		self.respFK = self.get_fk(self.seed)
		#wait for the service to respond and complete the requested calculation
		rospy.wait_for_service(self.fk_service)
		#store the robots position and orientation data from pose message into a new variable
		self.fkin = self.respFK.poses[0]
		#obtain the robot's end-effector position in the x direction of the robot's base frame
		x = self.fkin.position.x
		#obtain the robot's end-effector position in the y direction of the robot's base frame
		y = self.fkin.position.y
		#obtain the robot's end-effector position in the z direction of the robot's base frame
		z = self.fkin.position.z
		#concatinate the x, y, and z position values into a list for ease of use
		self.pos_data = [x, y, z]
		#set the output value of callback function to be the list containing the x, y, and z position of the robot's end-effector in the robot's base frame
		return self.pos_data
		
	#callback function for the data update timer (event is variable used for bookkeeping purposes and holds no information)
	def trial_timer_callback(self, event):
		#get current time of the system
		mytime = rospy.Time.now()
		#call the function for obtaining the forward kinematics service data
		pos_val = self.collect_fk_data()
		#concatinate the current system time and the foward kinematics x, y, and z positions
		values = [self.data_time, pos_val[0], pos_val[1], pos_val[2]]

		#update the data recording variable with trial conditions and collected experimental data
		self.mydata.data = values
		#publish the collected experimental data and trial conditions to the topic used for data recording
		if (self.Star_stop_index == 1 or (self.Sviz_con == 1 and self.Star_stop_index == 0) or (self.Sviz_con == 0 and self.Star_stop_index == 0)):
			
			self.record_data.publish(self.mydata)
		
		return self.data_time;
	

	
#Code used to control the ROS Node
def main():
	
	#intialize ros python node
	rospy.init_node('initial_node', anonymous=False, log_level=rospy.INFO, disable_signals=False)
	
	#intialize Control class variable for the node
	test = Control()
	
	i = 0
	try:
		while not rospy.is_shutdown():
			#check if all trials have been completed
			rospy.loginfo(i)
			if (i > len(data)-1):
				rospy.loginfo("Testing has been completed")
				rospy.signal_shutdown("Task has been completed")
				break;

			#starting condition runs for a time
			#variable used for storing the updating time while the data collection event is in progress
			check_time = rospy.Time.now()
			check_time = check_time.secs
			#variable used for storing starting time of data collection event in seconds
			start_time = rospy.Time.now()
			start_time = start_time.secs
			
			end_time = round(random.uniform(1.5,3.5), 1)
			#end_time = 2
			#print(end_time)
			while check_time - start_time <= end_time:
				#update the system time using data log time stamp
				check_time = rospy.Time.now()
				check_time = check_time.secs
				test.data_time = check_time
				print(check_time-start_time)
				

			
		#Begin Data Recording
		#data recording
			test.reset_timer()
			rospy.Rate(100)
			while k <= 2:
			#obtain forward kinematics data
				resp = get_fk(joints=seed)
				sol_pose = resp.poses[0]
			#combine all the needed data into on list
				desried_data = [running.data, movement_con, sol_pose]
				test.mydata.data = desired_data
			#publish the recorded data to a topic to be sent for data logging
				test.record_data.publish(test.mydata)
				
				rospy.Rate.sleep()
			
			#variable used for storing the updating time while the data collection event is in progress
			check_time = rospy.Time.now()
			check_time = check_time.secs
			#variable used for storing starting time of data collection event in seconds
			start_time = rospy.Time.now()
			start_time = start_time.secs
			#while loop conditional statement used to ensure the collection period lasts for 2 seconds
			
			while check_time - start_time <= 2:
				#update the system time using data log time stamp
				check_time = rospy.Time.now()
				check_time = check_time.secs
				test.data_time = check_time
				print(check_time-start_time)
			
			
		#stop data recording 
			
			i = i + 1
			
	except rospy.ROSInterruptException: 
		pass
	rospy.signal_shutdown("Recording has been completed")
	rospy.spin()
	
#running the ros nod
if __name__=='__main__':
    main()