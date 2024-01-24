import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import rospkg


rospy.init_node('spawn_drones',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 5

f = open(rospkg.RosPack().get_path('dronesim')+'/scripts/drone.sdf')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("drone_0", sdff, "robotos_name_space", initial_pose, "world")