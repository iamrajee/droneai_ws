import rospy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState, GetModelState
from std_msgs.msg import Int16, Bool
import sys


import math

import numpy as np

def quaternion_from_twist(modelstate):
    vx,vy = modelstate.twist.linear.x, modelstate.twist.linear.y
    theta = math.atan2(vy, vx)
    # Assuming roll and pitch are both 0
    # roll = 0
    # pitch = 0
    print("angle:", math.degrees(theta))
    # Convert angles to radians
    roll_rad = 0#np.radians(roll)
    pitch_rad = 0#np.radians(pitch)
    yaw_rad = theta#np.radians(yaw)

    # Calculate quaternion components
    qw = np.cos(yaw_rad/2)
    qx = 0#np.sin(yaw_rad/2) * np.cos(roll_rad) * np.cos(pitch_rad)
    qy = 0#np.sin(yaw_rad/2) * np.sin(roll_rad) * np.cos(pitch_rad)
    qz = np.sin(yaw_rad/2) #* np.sin(pitch_rad)

    # Return the quaternion
    print("quat:", qw, qx, qy, qz)
    return np.array([qw, qx, qy, qz])

def generate_circular_trajectory(radius, num_points):
  theta = 0
  points = []
  for i in range(num_points):
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    points.append((x, y))
    theta += 2 * math.pi / num_points  # Increment angle for next point
  return points

def generate_zigzag_circular_trajectory(radius, num_points):
    points = []
    for i in range(num_points):
        theta = 2 * math.pi * i / num_points
        x = radius * math.cos(theta)
        y = radius * math.sin(theta) * math.sin(2 * math.pi * i / 5)  # Sinusoidal zig-zag
        points.append((x, y))
    return points

radius = 3
num_points = 100  # Adjust for desired number of points
trajectory = generate_circular_trajectory(radius, num_points)
lenfactor = 5 
trajectory = [[lenfactor,lenfactor], [lenfactor,-lenfactor], [-lenfactor,-lenfactor], [-lenfactor,lenfactor] ]
# trajectory = generate_zigzag_circular_trajectory(radius, num_points)
trajectory_itr=0
num_points = len(trajectory)



drone_id = "0"

pub = rospy.Publisher('/move_command', Pose, queue_size=1)  # Create publisher for move_cb

drone_cords_offset = [[0.5,0.5,0.5],[0.5,-0.5,0.5],[-0.5,0.5,0.5],[-0.5,-0.5,0.5]]

number_of_drones=4

# Ring
drone_id_of_interest=(int(drone_id)%number_of_drones) + 1

# star
# drone_id_of_interest= 1

def update_position():
    
    global num_points
    global trajectory
    global trajectory_itr
    global lenfactor


    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    
    model_state = ModelState()
    model_state.model_name = "drone_0"
    print("trajectory_itr:",trajectory_itr)
    current_model_state = get_model_state(model_state.model_name, "")
    currx, curry = current_model_state.pose.position.x, current_model_state.pose.position.y
    trajectory_itr=trajectory_itr%num_points
    x,y = trajectory[trajectory_itr]
    xvel=x-currx
    yvel=y-curry

    vfactor=0.1*lenfactor
    
    model_state.pose.position.x = currx#x
    model_state.pose.position.y = curry#y
    model_state.pose.position.z = 5
    model_state.twist.linear.x = xvel/(xvel**2+yvel**2)**(1/2)*vfactor
    model_state.twist.linear.y = yvel/(xvel**2+yvel**2)**(1/2)*vfactor
    
    q = quaternion_from_twist(model_state)
    model_state.pose.orientation.w = q[0]
    model_state.pose.orientation.x = q[1]
    model_state.pose.orientation.y = q[2]
    model_state.pose.orientation.z = q[3]
    
    if xvel**2 < 0.25 and yvel**2 < 0.25:
        trajectory_itr+=1    

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(model_state)
    

    if not resp:
        print("Object cannot be moved")
        
        
        
    suv_model_state=model_state
    suv_model_state.model_name = "suv"
    suv_model_state.pose.position.z = 0
    resp = set_state(suv_model_state)
    if not resp:
        print("Object cannot be moved")
        
    marker_model_state=model_state
    marker_model_state.model_name = "marker_1"
    marker_model_state.pose.position.z = 2.5
    resp = set_state(marker_model_state)
    if not resp:
        print("Object cannot be moved")
        
    camera_model_state=model_state
    camera_model_state.model_name = "camera_top_left"
    # camera_model_state = get_model_state("camera_top_left", "")
    # camera_model_state.pose.position = model_state.pose.position
    # camera_model_state.twist = model_state.twist
    # camera_model_state.pose.position.z -=0.1
    # camera_model_state.pose.position.z -=0.1
    camera_model_state.pose.orientation.w, camera_model_state.pose.orientation.x, camera_model_state.pose.orientation.y, camera_model_state.pose.orientation.z = 0.7071, 0, 0.7071, 0
    suv_model_state.pose.position.z = 5
    resp = set_state(camera_model_state)
    if not resp:
        print("Object cannot be moved")
        
        


    # pub_odom.publish(msg.pose.pose)  # Publish the pose


def main():
    rospy.init_node('move_' + drone_id + "_in_gazebo", log_level=rospy.INFO)
    # spawn_drone()

    # pub_odom = rospy.Publisher('/drone_' + drone_id + '_visual_slam/odom', Odometry)
    while not rospy.is_shutdown():
        print("--------------drone:", drone_id)
        update_position()
        rospy.sleep(0.2)
        
    rospy.spin()

if __name__ == '__main__':
    main()
