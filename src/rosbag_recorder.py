#!/usr/bin/env python

import rospy
import roslaunch
from time import sleep
from std_srvs.srv import SetBool
import rospkg

import ConfigParser
import os

class Recorder:

	def __init__(self):
		self.package = 'rosbag'
		self.executable = 'record'
		self.request = False
		self.recording_started = False
		self.node_name = rospy.get_name()

		rospack = rospkg.RosPack()
		packagePath = rospack.get_path('rosbag_recorder')

		self.topicOption = rospy.get_param('/rosbag_recorder/topics')
		
		if not(rospy.get_param('/rosbag_recorder/rosbagPath') == ""):
			self.recordingPath = rospy.get_param('/rosbag_recorder/rosbagPath')
		else:
			self.recordingPath = packagePath + '/recordings/'
			if not (os.path.isdir(self.recordingPath)):
				print('filepath does not exist! Creating a new folder')
				os.makedirs(self.recordingPath)

		self.prepare_options()

	def prepare_options(self):
		list = os.listdir(self.recordingPath) # dir is your directory path
		number_files = len(list)
		self.filename = '/recording_' + str(number_files) + '.bag'
		self.rosbagArgs = 'record ' + self.topicOption + ' -o ' + self.recordingPath # + self.filename
		print('args are: ' + self.rosbagArgs)

	def start_recording(self):
		# first, update filename
		self.prepare_options()

		node = roslaunch.core.Node(self.package, self.executable, args = self.rosbagArgs)
		launch = roslaunch.scriptapi.ROSLaunch()
		launch.start()
		self.process = launch.launch(node)
		
		if (self.process.is_alive()):
			print('successfully started recording')

	def stop_recording(self):
		self.process.stop()
		if not (self.process.is_alive()):
			print('successfully stopped recording')

	def start_service(self):
		service_topic = self.node_name + '/recorder'
		self.server = rospy.Service(service_topic, SetBool, self.handle_service_call)

	def handle_service_call(self,req):
		if (req.data):
			self.request = True
			return (True, 'started recording to ' + self.recordingPath + self.filename)
		else:
			self.request = False
			return (True, 'stopped recording')

	def check_if_true(self):
		while 1:
			if (self.request and not self.recording_started):
				self.recording_started = True
				sleep(1)
				self.start_recording()
			elif (not self.request and not self.recording_started):
				sleep(1)
			elif (not self.request and self.recording_started):
				self.recording_started = False
				self.stop_recording()
				sleep(1)
			else:
				sleep(1)
				pass
				
def main():
	rospy.init_node('~', anonymous=True)

	recorder = Recorder()
	recorder.start_service()
	recorder.check_if_true()

	rospy.spin()

if __name__ == '__main__':
	main()
