import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from utils.tools import *

def navigate_base_to_pick_position(p, robot, mug_position):   
    tolerance = 0.05
    mug_x, mug_y, mug_z = mug_position

    current_position, _, current_orientation = get_robot_base_pose(p, robot.robotId)
    current_x, current_y = current_position[0], current_position[1]
    target_x, target_y, target_h = mug_x-0.08, 0.5, mug_z + 0.04

    # move arm up to avoid collision
    arm_position,_,_ = get_robot_ee_pose(p,robot.robotId)
    arm_h = arm_position[2] 
    #print(arm_position)   
    
    while(arm_h<1.1):
        #up_direction = 1 if target_h-arm_h > 0 else -1     
        arm_control(robot, p, 2, 0, 0, 0)
        p.stepSimulation()
        robot.get_observation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,robot.robotId)
        arm_h = arm_position[2]       
    arm_control(robot, p, 0, 0, 0, 0)        
    p.stepSimulation()

    # turn to face Y direction
    target_yaw_y = -math.pi / 2 if target_y < current_y else math.pi / 2
    while not turn_to_direction(p, robot, target_yaw_y):
        p.stepSimulation()
        robot.get_observation()
        time.sleep(0.01)

    # move to target Y position
    while abs(current_y - target_y) > tolerance:
        base_control(robot, p, forward=0.5, turn=0)
        p.stepSimulation()
        robot.get_observation()
        time.sleep(0.01)
        current_position, _, _ = get_robot_base_pose(p, robot.robotId)
        current_y = current_position[1]
    base_control(robot, p, forward=0, turn=0)

    # turn to face X direction
    target_yaw_x = 0 if target_x > current_x else math.pi
    while not turn_to_direction(p, robot, target_yaw_x):
        p.stepSimulation()
        robot.get_observation()
        time.sleep(0.01)

    # move to target X position
    while abs(current_x - target_x) > tolerance:
        base_control(robot, p, forward=0.5, turn=0)
        p.stepSimulation()
        robot.get_observation()
        time.sleep(0.01)
        current_position, _, _ = get_robot_base_pose(p, robot.robotId)
        current_x = current_position[0]
    base_control(robot, p, forward=0, turn=0)

    print("Reached picking position:", current_position)
    return True

def turn_to_direction(p, robot, target_yaw, tolerance=0.05, turning_speed=0.5):    
    _, _, current_orientation = get_robot_base_pose(p, robot.robotId)
    current_yaw = current_orientation[2]  # Extract yaw from Euler angles

    # calculate yaw error
    yaw_error = target_yaw - current_yaw

    # normalize yaw error to range [-pi, pi]
    yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

    if abs(yaw_error) > tolerance:
        # determine turning direction (positive or negative velocity)
        turning_direction = 1 if yaw_error > 0 else -1
        base_control(robot, p, forward=0, turn=turning_speed * turning_direction)
        return False  # Task not yet complete
    else:
        # stop turning when within tolerance
        base_control(robot, p, forward=0, turn=0)
        return True
    
def move_to_target_y(p, robot, target_y, tolerance=0.1):
    target_yaw_y = -math.pi / 2 if target_y < get_robot_base_pose(p, robot.robotId)[0][1] else math.pi / 2
    while not turn_to_direction(p, robot, target_yaw_y):
        p.stepSimulation()
    while abs(get_robot_base_pose(p, robot.robotId)[0][1] - target_y) > tolerance:
        base_control(robot, p, forward=0.5, turn=0)
        p.stepSimulation()
    base_control(robot, p, forward=0, turn=0)

def move_to_target_x(p, robot, target_x, tolerance=0.1):
    target_yaw_x = 0 if target_x > get_robot_base_pose(p, robot.robotId)[0][0] else math.pi
    while not turn_to_direction(p, robot, target_yaw_x):
        p.stepSimulation()
    while abs(get_robot_base_pose(p, robot.robotId)[0][0] - target_x) > tolerance:
        base_control(robot, p, forward=0.2, turn=0)
        p.stepSimulation()
    base_control(robot, p, forward=0, turn=0)

def pick_and_place_mug(p, robot, mug_position):
    mug_x, mug_y, mug_z = mug_position
    arm_position,_,_ = get_robot_ee_pose(p,robot.robotId)    
    target_y, target_h = mug_y, mug_z+0.05

    arm_position,_,_ = get_robot_ee_pose(p,robot.robotId)
    arm_y = arm_position[1]    
    
    # reach out to put ee above mug
    while(abs(target_y-arm_y)>0.01):        
        arm_control(robot, p, 0, 1, 0, 0)
        robot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,robot.robotId)
        arm_y = arm_position[1]        
    arm_control(robot, p, 0, 0, 0)

    # lower ee to right above mug
    arm_position,_,_ = get_robot_ee_pose(p,robot.robotId)
    arm_h = arm_position[2]    
    
    while(abs(target_h-arm_h) > 0.05):
        up_direction = 1 if target_h-arm_h > 0 else -1    
        arm_control(robot, p, up_direction*0.5, 0, 0, 0)
        robot.get_observation()
        p.stepSimulation()
        time.sleep(0.01)
        arm_position,_,_ = get_robot_ee_pose(p,robot.robotId)
        arm_h = arm_position[2]
      
    arm_control(robot, p, 0, 0, 0)

    # pick mug and lift it up a bit    
    constraint = attach(21, robot.robotId, 18)
    if constraint is not None:    
        for i in range(0,2):
            arm_control(robot, p, 1, 0, 0, 0)
            robot.get_observation()
            p.stepSimulation()
            time.sleep(0.01)
        arm_control(robot, p, 0, 0, 0, 0)

        # move to drawer
        current_position, _, current_orientation = get_robot_base_pose(p, robot.robotId)
        current_x = current_position[0]
        target_x = 3.36

        while(abs(target_x - current_x)>0.05):
            base_control(robot, p, forward= -0.5, turn=0)
            robot.get_observation()
            p.stepSimulation()
            time.sleep(0.1)
            current_position, _, _ = get_robot_base_pose(p, robot.robotId)
            current_x = current_position[0]
        base_control(robot, p, forward=0, turn=0)
        
        # lower mug to be inside drawer        
        _,_, mug_z = get_mug_pose(p)
        while(mug_z >=0.75):
            arm_control(robot, p, -0.3, 0, 0, 0)
            robot.get_observation()
            p.stepSimulation()
            time.sleep(0.1)
            _,_, mug_z = get_mug_pose(p)
        arm_control(robot, p, 0, 0, 0, 0)
        p.stepSimulation()
        time.sleep(0.1)
        
    return constraint

