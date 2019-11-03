# rosbag_recorder

Rosbag_recorder is a simple wrapper that lets you start and stop rosbag record with a single service call. Just call '/recorder' with argument 'true' and it will start recording to the path specified in the config (settings.cfg). Calling the service with argument 'false' will stop the recording and rosbag_recorder will wait in the background for another call.

Requirements:
1. ROS

Usage:

- specify topics and path in settings.cfg
- start recording with following command:

`roslaunch rosbag_recorder rosbag_recorder.launch`
