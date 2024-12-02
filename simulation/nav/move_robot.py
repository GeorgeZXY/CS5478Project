import math
import time
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from utils.tools import *

class RobotController:
    def __init__(self, mobot, start_pose, isReverse):
        self.robotId = mobot.robotId
        self.start_pose = start_pose
        self.current_base_yaw = self.start_pose[1][2]
        self.isReverse = isReverse
        self.timestep = 0.01  # set duration of each step
        self.distance_controller = PIDController(0.01, 0.0, 0.0, 0.0)  # initialize PID controller
        #self.mobot = mobot

    def sim_get_current_base_pose(self):
        base_position, base_orientation = p.getBasePositionAndOrientation(self.robotId)
        # base_orientation_eula = R.from_quat(base_orientation).as_euler("xyz", degrees=False)
        base_orientation_eula = p.getEulerFromQuaternion(base_orientation)
        return base_position, base_orientation, base_orientation_eula

    def run(self, steps=1):
        for _ in range(steps):
            #self.mobot.get_observation()
            p.stepSimulation()
            time.sleep(self.timestep)

    def sim_rotate_base_to_target_yaw(self, target_yaw, gradual=True, step_size=0.02):
        if self.isReverse:
            target_yaw = (target_yaw + math.pi) % (2 * math.pi)

        def angle_to_quaternion(yaw):
            return [0, 0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

        def shortest_angular_distance(from_angle, to_angle):
            return (to_angle - from_angle + math.pi) % (2 * math.pi) - math.pi

        angle_diff = shortest_angular_distance(self.current_base_yaw, target_yaw)
        if gradual:
            while abs(angle_diff) > step_size:
                self.current_base_yaw += step_size if angle_diff > 0 else -step_size
                self.current_base_yaw = (self.current_base_yaw + math.pi) % (2 * math.pi) - math.pi
                angle_diff = shortest_angular_distance(self.current_base_yaw, target_yaw)
                orientation = angle_to_quaternion(self.current_base_yaw)
                position, _ = p.getBasePositionAndOrientation(self.robotId)
                p.resetBasePositionAndOrientation(self.robotId, position, orientation)
                self.run()
        else:
            orientation = angle_to_quaternion(target_yaw)
            position, _ = p.getBasePositionAndOrientation(self.robotId)
            p.resetBasePositionAndOrientation(self.robotId, position, orientation)
            self.run(5)
            
    def sim_action(self, output):
        if self.isReverse:
            output = -output

        position, orientation = p.getBasePositionAndOrientation(self.robotId)
        euler_angles = p.getEulerFromQuaternion(orientation)
        # print("Euler angles: ", euler_angles)
        # e2 = get_robot_base_pose(p, self.robotId)
        # print("E2: ", e2)
        
        p.resetBasePositionAndOrientation(
            self.robotId,
            [position[0] + output * math.cos(euler_angles[2]), 
             position[1] + output * math.sin(euler_angles[2]), 
             position[2]],
            orientation
        )

    def sim_move_base_to_waypoint(self, waypoint, threshold=0.01):
        self.target_distance = 0.0
        self.base_rotated = False
        cnt = 1

        while True:
            base_position, base_orientation, base_orientation_eula = self.sim_get_current_base_pose()
            #print("Base position: ", base_position)
            #print("Base orientation: ", base_orientation)
            # base_orientation_eula = R.from_quat(base_orientation).as_euler("xyz", degrees=False)
            target = waypoint
            #print("Target: ", target)
            # distance = np.linalg.norm(np.array([base_position[0], base_position[1]]) - \
            #                           np.array([waypoint[0][0], waypoint[0][1]]))
            distance = math.sqrt((target[0][1] - base_position[1]) ** 2 + (target[0][0] - base_position[0]) ** 2)
            if distance < threshold:
                break

            self.distance_controller.set_goal(self.target_distance)
            output = self.distance_controller.calculate(distance)
            yaw = math.atan2(target[0][1] - base_position[1], 
                             target[0][0] - base_position[0])

            if not self.base_rotated:
                self.sim_rotate_base_to_target_yaw(yaw)
                self.base_rotated = True

            self.sim_action(-output)

            if cnt % 20 == 0:
                self.run()

            cnt += 1

        self.run()

    def sim_navigate_base(self, goal_base_pose, path, threshold=0.05):
        for i in range(len(path)):
            waypoint = [path[i][0], path[i][1], 0]
            print ("Moving to waypoint ", waypoint)
        # for waypoint in path:
            self.sim_move_base_to_waypoint(([waypoint[0], waypoint[1], 0], 
                                                 goal_base_pose[1]))
            
        self.sim_rotate_base_to_target_yaw(goal_base_pose[1][2])
        self.run(10)
            
        ik_error = self.sim_calculate_nav_error(goal_base_pose)
        if ik_error >= threshold:
            print(f"Warning: Navigation error: {ik_error}")
        print("Navigation is done!")

    def sim_calculate_nav_error(self, goal_pose):
        current_base_pose = self.sim_get_current_base_pose()
        distance = np.linalg.norm(np.array([current_base_pose[0][0],
                                            current_base_pose[0][1],
                                            0.05]) - \
            np.array(goal_pose[0]))
        return distance

# a simplified PID controller
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0

    def calculate(self, process_value):
        error = self.setpoint - process_value
        self.integral += error
        derivative = error - self.last_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

    def set_goal(self, setpoint):
        self.setpoint = setpoint
