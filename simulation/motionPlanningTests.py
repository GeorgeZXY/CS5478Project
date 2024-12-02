import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from utils.tools import *
from nav.map import *
from nav.A_star import *
from nav.move_robot import *
from scipy.spatial.transform import Rotation as R

tolerance = 0.05
target1 = [0.27, -0.71, 0.92]
target2 = [-1.70, -3.70, 0.46]
target3 = [1.45, -1.68, 0.59]

def display_points(points, labels=True, color=[1, 0, 0], line_width=3, text_size=1.0):
    # displays specified points in the simulation environment with optional labels.
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

def move_To_Target1(mobot, nav_map):
    beg_pos, base_ori = p.getBasePositionAndOrientation(mobot.robotId)
    base_ori_eula = R.from_quat(base_ori).as_euler("xyz", degrees=False)
    beg_x, beg_y, beg_z = beg_pos

    base_start_pos = [[beg_x, beg_y, beg_z],
                [base_ori_eula[0], base_ori_eula[1], base_ori_eula[2]]]    
    base_end_pose = [[-0.3,-0.71,0], [0, 0, math.pi/2]]

    robot_size = 0.5
    nav_planner = AStarPlanner(
        robot_size=robot_size,
        obstacles_bounds=nav_map,
        resolution=0.05,
        enable_plot=False
    )

    # move to a position that is close to target1
    target1Base = [-0.3, -0.7, 0]

    path = nav_planner.plan(
        start_pos=beg_pos, goal_pos=target1Base
    )
    #print("Path: ", path)

    controller = RobotController(mobot, base_start_pos, False)
    controller.sim_navigate_base(base_end_pose, path)

    tx, ty, tz = target1[0], target1[1], target1[2]

    # raise arm first to avoid collision
    arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)     
    while(arm_position[2]<1.1):    
        arm_control(mobot, p, 2, 0, 0, 0)
        p.stepSimulation()
        mobot.get_observation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)      
    arm_control(mobot, p, 0, 0, 0, 0)        
    p.stepSimulation()

    # adjust ee y to taget y
    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    while abs(ee_position[1] - ty) > tolerance:
        direction = 1 if ee_position[1] < ty else -1 
        base_control(mobot, p, forward=0.5*direction, turn=0)
        p.stepSimulation()
        mobot.get_observation()
        time.sleep(0.01)
        ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)        
    base_control(mobot, p, forward=0, turn=0)

    # reach out to put ee at target x
    arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)    
    init_arm_pos = arm_position[0]

    while(abs(tx-arm_position[0])>tolerance):        
        arm_control(mobot, p, 0, 1, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)                
    arm_control(mobot, p, 0, 0, 0)

    # lower ee to target z
    arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)   
    while(abs(tz-arm_position[2]) > tolerance):
        up_direction = 1 if tz > arm_position[2] else -1    
        arm_control(mobot, p, up_direction*0.5, 0, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)   
    arm_control(mobot, p, 0, 0, 0)

    # run motion planning test
    print("Testing target point 1", target1)
    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    print("End-effector position: ", ee_position)
    motion_planning_test(p, mobot.robotId, target1)
    p.stepSimulation()
    time.sleep(3)

    # retract arm
    while(arm_position[0]>init_arm_pos):        
        arm_control(mobot, p, 0, -1, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)                
    arm_control(mobot, p, 0, 0, 0)

def move_To_Target2(mobot, nav_map):
    beg_pos, base_ori = p.getBasePositionAndOrientation(mobot.robotId)
    base_ori_eula = R.from_quat(base_ori).as_euler("xyz", degrees=False)
    beg_x, beg_y, beg_z = beg_pos
    base_start_pos = [[beg_x, beg_y, beg_z],
                [base_ori_eula[0], base_ori_eula[1], base_ori_eula[2]]]    
    base_end_pose = [[-1.4, -3.4, 0], [0, 0, -math.pi/2]]

    robot_size = 0.5 #sim_get_robot_size(mobot) 0.5
    nav_planner = AStarPlanner(
        robot_size=robot_size,
        obstacles_bounds=nav_map,
        resolution=0.05,
        enable_plot=False
    )

    # move to a position that is close to target2
    target2Base = [-1.4, -3.4, 0]

    path = nav_planner.plan(
        start_pos=beg_pos, goal_pos=target2Base
    )
    #print("Path: ", path)

    controller = RobotController(mobot, base_start_pos, False)
    controller.sim_navigate_base(base_end_pose, path)
    p.stepSimulation()
    time.sleep(0.01)

    tx, ty, tz = target2[0], target2[1], target2[2]

    # adjust ee y to taget y
    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    while abs(ty - ee_position[1]) > tolerance:
        direction = 1 if ee_position[1] > ty else -1 
        base_control(mobot, p, forward=0.3*direction, turn=0)
        p.stepSimulation()
        mobot.get_observation()
        time.sleep(0.01)
        ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)        
    base_control(mobot, p, forward=0, turn=0)

    # reach out to put ee at target x
    arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)    
    init_arm_pos = arm_position[0]
    while(abs(tx-arm_position[0])>tolerance):        
        arm_control(mobot, p, 0, 1, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)                
    arm_control(mobot, p, 0, 0, 0)

    # adjust ee to target z
    arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)    
    while(abs(tz-arm_position[2]) > tolerance):
        up_direction = 1 if tz > arm_position[2] else -1    
        arm_control(mobot, p, up_direction*1, 0, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)   
    arm_control(mobot, p, 0, 0, 0)

    # run motion planning test
    print("Testing target point 2", target2)
    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    print("End-effector position: ", ee_position)
    motion_planning_test(p, mobot.robotId, target2)
    p.stepSimulation()
    time.sleep(3)

    # retract arm
    while(arm_position[0] < init_arm_pos):        
        arm_control(mobot, p, 0, -1, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)                
    arm_control(mobot, p, 0, 0, 0)

def move_To_Target3(mobot, nav_map):
    # raise arm first to avoid collision
    arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)     
    while(arm_position[2]<0.8):    
        arm_control(mobot, p, 2, 0, 0, 0)
        p.stepSimulation()
        mobot.get_observation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)      
    arm_control(mobot, p, 0, 0, 0, 0)        
    p.stepSimulation()

    beg_pos, base_ori = p.getBasePositionAndOrientation(mobot.robotId)
    base_ori_eula = R.from_quat(base_ori).as_euler("xyz", degrees=False)
    beg_x, beg_y, beg_z = beg_pos
    base_start_pos = [[beg_x, beg_y, beg_z],
                [base_ori_eula[0], base_ori_eula[1], base_ori_eula[2]]]    
    base_end_pose = [[1.5, -2.2, 0], [0, 0, math.pi]]

    robot_size = 0.5 #sim_get_robot_size(mobot) 0.5
    nav_planner = AStarPlanner(
        robot_size=robot_size,
        obstacles_bounds=nav_map,
        resolution=0.05,
        enable_plot=False
    )

    # move to a position that is close to target3
    target3Base = [1.5, -2.2, 0]

    path = nav_planner.plan(
        start_pos=beg_pos, goal_pos=target3Base
    )
    #print("Path: ", path)

    controller = RobotController(mobot, base_start_pos, False)
    controller.sim_navigate_base(base_end_pose, path)
    p.stepSimulation()
    time.sleep(0.01)

    tx, ty, tz = target3[0], target3[1], target3[2]

    # adjust ee x to taget x
    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    while abs(tx - ee_position[0]) > tolerance:
        direction = 1 if ee_position[0] > tx else -1 
        base_control(mobot, p, forward=0.3*direction, turn=0)
        p.stepSimulation()
        mobot.get_observation()
        time.sleep(0.01)
        ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)        
    base_control(mobot, p, forward=0, turn=0)

    # reach out to put ee at target y
    arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)    
    init_arm_pos = arm_position[1]
    while(abs(ty-arm_position[1])>tolerance):      
        arm_control(mobot, p, 0, 1, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)                
    arm_control(mobot, p, 0, 0, 0)

    # adjust ee to target z
    arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)    
    while(abs(tz-arm_position[2]) > tolerance):
        up_direction = 1 if tz > arm_position[2] else -1    
        arm_control(mobot, p, up_direction*0.5, 0, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)   
    arm_control(mobot, p, 0, 0, 0)

    # run motion planning test
    print("Testing target point 3", target3)
    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    print("End-effector position: ", ee_position)
    motion_planning_test(p, mobot.robotId, target3)
    p.stepSimulation()
    time.sleep(3)

    # retract arm
    while(arm_position[1]>init_arm_pos):        
        arm_control(mobot, p, 0, -1, 0, 0)
        mobot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,mobot.robotId)                
    arm_control(mobot, p, 0, 0, 0)