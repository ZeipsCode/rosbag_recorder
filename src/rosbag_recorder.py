#!/usr/bin/env python

import rospy
import roslaunch
from time import sleep
from std_srvs.srv import SetBool
import rospkg

import ConfigParser
import os

class Recorder:

	def __init__(self,options,path):
		self.options = options
		self.path = path
		self.package = 'rosbag'
		self.executable = 'record'
		self.request = False
		self.recording_started = False

		rospack = rospkg.RosPack()
		packagePath = rospack.get_path('rosbag_recorder')

		configPath = os.path.join(packagePath,'config' , 'settings.cfg')

		# read config file
		config = ConfigParser.ConfigParser()
		config.readfp(open(configPath))

		self.topicOption = config.get('recordingOptions' , 'topics')
		
		try:
			self.recordingPath = config.get('recordingOptions', 'rosbagPath')
		except Exception as e:
			self.recordingPath = packagePath
			print('no path specified...Recording to : ', self.recordingPath)
		else:
			pass
		finally:
			self.prepare_options()		

	def prepare_options(self):
		list = os.listdir(self.recordingPath) # dir is your directory path
		number_files = len(list)
		self.filename = '/recording_' + str(number_files) + '.bag'
		self.rosbagArgs = 'record ' + self.topicOption + ' -O ' + self.recordingPath + self.filename  #'record --all -O /home/dennis/bagfiles/test.bag'

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
		self.server = rospy.Service('recorder', SetBool, self.handle_service_call)

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
	rospy.init_node('rosbag_recorder', anonymous=True)

	recorder = Recorder('a','b')
	recorder.start_service()
	recorder.check_if_true()

	rospy.spin()

if __name__ == '__main__':
	main()
