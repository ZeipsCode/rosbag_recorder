# rosbag_recorder

rosbag_recorder is a simple wrapper that lets you start and stop rosbag record with a single service call. Just call '/recorder' with argument 'true' and it will start recording to the path specified in the config (settings.cfg). Calling the service with argument 'false' will stop the recording and rosbag_recorder will wait in the background for another call.

Requirements:
1. ROS

Usage:

- specify topics and path in rosbag_recorder.launch like in the example below, where a recording of topics `/test1` and `/test2` will be started:

```xml
<launch>  
  <node pkg="rosbag_recorder" type="rosbag_recorder.py" name="rosbag_recorder" output="screen" > 
  	<param name="topics" value="/test1 /test2" />
  	<param name="rosbagPath" value=" " />
  </node>
</launch>
```

- start recording with following command:

`roslaunch rosbag_recorder rosbag_recorder.launch`

Launching that node will start a service server that can be used to trigger and stop a recording. For example you can start the recording by tiping the following command into the console:

`rosservice call /rosbag_recorder/recorder "data: true"`

And stopping works like this:

`rosservice call /rosbag_recorder/recorder "data: false"`
