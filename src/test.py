import roslaunch
from time import sleep

package = 'rosbag'
executable = 'record'
node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print('recording in process? ', process.is_alive())

sleep(5)

print('ok, enough...killing that process...')

process.stop()