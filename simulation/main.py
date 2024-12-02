import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from utils.tools import *
from pickMug import *

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -9.81)

#true represent place a cup randomly
mobot = init_scene(p, mug_random=True)

def display_points(points, labels=True, color=[1, 0, 0], line_width=3, text_size=1.0):
    """
    Displays specified points in the simulation environment with optional labels.
    
    :param points: List of points (each as [x, y, z]) to display.
    :param labels: Whether to add text labels for the points.
    :param color: RGB color of the points or markers.
    :param line_width: Line width for visualizing points.
    :param text_size: Size of the text labels.
    """
    for i, point in enumerate(points):
        # Draw a small cross at the point using lines
        offset = 0.1  # Size of the cross
        x, y, z = point
        
        # X-axis line
        p.addUserDebugLine([x - offset, y, z], [x + offset, y, z], color, lineWidth=line_width)
        # Y-axis line
        p.addUserDebugLine([x, y - offset, z], [x, y + offset, z], color, lineWidth=line_width)
        # Z-axis line
        p.addUserDebugLine([x, y, z - offset], [x, y, z + offset], color, lineWidth=line_width)
        
        # Add text label if enabled
        if labels:
            p.addUserDebugText(f"Point {i+1}", [x, y, z], textColorRGB=color, textSize=text_size)

points_to_display = [
    [0.27, -0.71, 0.92],
    [-1.70, -3.70, 0.46],
    [1.45, -1.68, 0.59]
]

#display_points(points_to_display)

def display_local_axes(object_id, length=0.1):
    """
    Displays the local coordinate axes of an object.
    :param object_id: The ID of the object to inspect.
    :param length: The length of the axes to draw.
    """
    # Get the base position and orientation of the object
    position, orientation = p.getBasePositionAndOrientation(object_id)
    
    # Convert orientation (quaternion) to rotation matrix
    rotation_matrix = p.getMatrixFromQuaternion(orientation)
    rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
    
    # Local axes in object frame
    local_axes = np.eye(3)  # Identity matrix for x, y, z axes
    
    # Global axes in world frame
    global_axes = np.dot(rotation_matrix, local_axes.T).T
    
    # Draw axes
    origin = np.array(position)
    for axis, color in zip(global_axes, [[1, 0, 0], [0, 1, 0], [0, 0, 1]]):  # Red, Green, Blue for X, Y, Z
        end = origin + length * axis
        p.addUserDebugLine(origin, end, lineColorRGB=color, lineWidth=3)

# display_local_axes(21)

forward=0
turn=0
speed=10
up=0
stretch=0
gripper_open=0
roll=0
yaw=0

mobot.get_observation()

#########################
############start##########
#########################

from nav.map import *
from nav.A_star import *
from nav.move_robot import *
from scipy.spatial.transform import Rotation as R


nav_map = get_navigation_map(mobot, True)

beg_pos, base_ori = p.getBasePositionAndOrientation(mobot.robotId)
base_ori_eula = R.from_quat(base_ori).as_euler("xyz", degrees=False)
beg_x, beg_y, beg_z = beg_pos
print("Initial position: ", beg_pos)
print("Initial orientation: ", base_ori) # Quaternion
beg_ori = list(base_ori)

beg_pose = ([beg_x, beg_y, beg_z], [beg_ori[0], beg_ori[1], beg_ori[2], beg_ori[3]])
base_pose1 = [[beg_x, beg_y, beg_z],
              [base_ori_eula[0], base_ori_eula[1], base_ori_eula[2]]]

mid_point = [1.785,-2.8,0]
mid_pose = [[1.785,-2.8,0], [0, 0, math.pi/2]]

goal_pose = [[2.85, 0.1, 0], [0, 0, math.pi/2]]
goal_pos = [2.85, 0.1, 0]



robot_size = 0.5 #sim_get_robot_size(mobot) 0.5
nav_planner = AStarPlanner(
    robot_size=robot_size,
    obstacles_bounds=nav_map,
    resolution=0.05,
    enable_plot=True
)



path = nav_planner.plan(
    start_pos=beg_pos, goal_pos=mid_point
)

print("Path: ", path)

controller = RobotController(mobot, base_pose1, False)
controller.sim_navigate_base(mid_pose, path)

#======

# Raise arm to avoid collision
timestep = 0.01
p.setJointMotorControl2(mobot.robotId,8,p.POSITION_CONTROL,targetPosition=0.65,force=100)
for _ in range(1):
    p.stepSimulation()
    time.sleep(timestep)

robot_size = 0.2 
nav_planner = AStarPlanner(
    robot_size=robot_size,
    obstacles_bounds=nav_map,
    resolution=0.05,
    enable_plot=True
)

path1 = nav_planner.plan(
    start_pos=mid_point, goal_pos=goal_pos
)

print("Path1: ", path1)

controller = RobotController(mobot, mid_pose , True)
controller.sim_navigate_base(goal_pose, path1)

#########################
############end##########
#########################

total_driving_distance = 0
previous_position, _, _ = get_robot_base_pose(p, mobot.robotId)
current_position = previous_position

constraint = None

navi_flag = False
grasp_flag = False

####？？？
first_time_reach = True
reached_pick_position = False
picked = False
placed = False
####？？？？

while (1):
    time.sleep(1./240.)
    keys = p.getKeyboardEvents()

    for k,v in keys.items():
        # moving
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            turn = -1
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
            turn = 0
        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            turn = 1
        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
            turn = 0
        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            forward=1
        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
            forward=0
        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            forward=-1
        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
            forward=0

        # lifting
        if (k == ord('z') and (v & p.KEY_WAS_TRIGGERED)):
            up = 1
        if (k == ord('z') and (v & p.KEY_WAS_RELEASED)):
            up = 0
        if (k == ord('x') and (v & p.KEY_WAS_TRIGGERED)):
            up = -1
        if (k == ord('x') and (v & p.KEY_WAS_RELEASED)):
            up = 0

        # stretching
        if (k == ord('a') and (v & p.KEY_WAS_TRIGGERED)):
            stretch = -1
        if (k == ord('a') and (v & p.KEY_WAS_RELEASED)):
            stretch = 0
        if (k == ord('d') and (v & p.KEY_WAS_TRIGGERED)):
            stretch = 1
        if (k == ord('d') and (v & p.KEY_WAS_RELEASED)):
            stretch = 0

        # roll
        if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
            roll = 1
        if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
            roll = 0
        if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
            roll = -1
        if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
            roll = 0

        # yaw
        if (k == ord('y') and (v & p.KEY_WAS_TRIGGERED)):
            yaw = 1
        if (k == ord('y') and (v & p.KEY_WAS_RELEASED)):
            yaw = 0
        if (k == ord('h') and (v & p.KEY_WAS_TRIGGERED)):
            yaw = -1
        if (k == ord('h') and (v & p.KEY_WAS_RELEASED)):
            yaw = 0


        # gripper
        if (k == ord('q') and (v & p.KEY_WAS_TRIGGERED)):
            gripper_open = -1
        if (k == ord('q') and (v & p.KEY_WAS_RELEASED)):
            gripper_open = 0
        if (k == ord('e') and (v & p.KEY_WAS_TRIGGERED)):
            gripper_open = 1
        if (k == ord('e') and (v & p.KEY_WAS_RELEASED)):
            gripper_open = 0

    base_control(mobot, p, forward, turn)
    arm_control(mobot, p, up, stretch, roll, yaw)

    if gripper_open == 1:
        constraint = attach(21, mobot.robotId, 18)
    elif gripper_open == -1:
        detach(constraint)
        constraint = None
    
    mobot.get_observation()
    
    
    ###link_orientation, euler_orientation和__
    current_position, link_orientation, euler_orientation = get_robot_base_pose(p, mobot.robotId)
    total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
    previous_position = current_position

    if navi_flag == False:
        if current_position[0] > 1.6 and current_position[1] > -0.35:
            print("Reached the goal region! Total driving distance: ", total_driving_distance)
            navi_flag = True
        else:
            print("Total driving distance: ", total_driving_distance)
            print("Current position: ", current_position)
    else:
        ####start####
        if first_time_reach:
            print("Reached the goal region! Total driving distance: ", total_driving_distance)
            first_time_reach = False
        ####end######
    
    
    if grasp_flag == False:
        mug_position = get_mug_pose(p)
        print("Mug position: ", mug_position)

        if mug_position[0] > 3.3 and mug_position[0] < 3.5 \
            and mug_position[1] > -0.17 and mug_position[1] < 0.25 \
            and mug_position[2] > 0.71 and mug_position[2] < 0.75:
            print("Mug is in the drawer!")
            grasp_flag = True
    else:
        print("Mug is in the drawer!")

    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    #print("End-effector position: ", ee_position)

    if not reached_pick_position:        
        reached_pick_position = navigate_base_to_pick_position(p, mobot, get_mug_pose(p))
        print("reached pick position")

    if not picked:
        constraint = pick_and_place_mug(p, mobot, get_mug_pose(p))
        picked = True

    if grasp_flag and not placed:        
        constraint = detach(constraint)  
        position, orientation = p.getBasePositionAndOrientation(21)
        # Slightly lower the object's position to separate it from the end effector
        new_position = [position[0] , position[1] , position[2]-0.02]
        p.resetBasePositionAndOrientation(21, new_position, orientation)
        p.stepSimulation()    
        placed = True

        for i in range(0,5):
            arm_control(mobot, p, 1, 0, 0, 0)
            mobot.get_observation()
            p.stepSimulation()
            time.sleep(0.1)
        arm_control(mobot, p, 0, 0, 0, 0)
