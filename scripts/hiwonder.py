# hiwonder.py
"""
Hiwonder Robot Controller
-------------------------
Handles the control of the mobile base and 5-DOF robotic arm using commands received from the gamepad.
"""

import time
from math import sin, cos, atan2, radians, degrees, sqrt, acos
import numpy as np
from ros_robot_controller_sdk import Board
from bus_servo_control import *

import utils as ut

# Robot base constants
WHEEL_RADIUS = 0.047  # meters
BASE_LENGTH_X = 0.096  # meters
BASE_LENGTH_Y = 0.105  # meters


class HiwonderRobot:
    def __init__(self):
        """Initialize motor controllers, servo bus, and default robot states."""
        self.board = Board()
        self.board.enable_reception()
        self.bsc = BusServoControl(self.board)

        # lengths of arm
        self.l1, self.l2, self.l3, self.l4, self.l5 = 0.155, 0.099, 0.095, 0.055, 0.105

        self.joint_values = [0, 0, 90, -30, 0, 0]  # degrees
        self.home_position = [0, 0, 90, -30, 0, 0]  # degrees
        self.joint_limits = [
            [-120, 120],
            [-90, 90],
            [-120, 120],
            [-100, 100],
            [-90, 90],
            [-120, 30],
        ]
        self.joint_control_delay = 0.2  # secs
        self.speed_control_delay = 0.2
        self.time_out = 100

        self.move_to_home_position()

    # -------------------------------------------------------------
    # Methods for interfacing with the mobile base
    # -------------------------------------------------------------

    def set_robot_commands(self, cmd: ut.GamepadCmds):
        """Updates robot base and arm based on gamepad commands.

        Args:
            cmd (GamepadCmds): Command data class with velocities and joint commands.
        """

        if cmd.arm_home:
            self.move_to_home_position()
        elif cmd.arm_j1:
            self.move_to_position_1()

        # print(f"---------------------------------------------------------------------")

        # self.set_base_velocity(cmd)
        # self.set_arm_velocity(cmd)

        ######################################################################

        position = [0] * 3

        ######################################################################

        # update joint values
        self.update_joint_values()

        # print(f'Joint values: {self.get_joint_values()}')

        print(
            f"[DEBUG] XYZ position: X: {round(position[0], 3)}, Y: {round(position[1], 3)}, Z: {round(position[2], 3)} \n"
        )

    def set_base_velocity(self, cmd: ut.GamepadCmds):
        """Computes wheel speeds based on joystick input and sends them to the board"""
        """
        motor3 w0|  ↑  |w1 motor1
                 |     |
        motor4 w2|     |w3 motor2
        
        """
        ######################################################################
        # insert your code for finding "speed"

        speed = [0] * 4

        ######################################################################

        # Send speeds to motors
        self.board.set_motor_speed(speed)
        time.sleep(self.speed_control_delay)

    # -------------------------------------------------------------
    # Methods for interfacing with the 5-DOF robotic arm
    # -------------------------------------------------------------

    def set_arm_velocity(self, cmd: ut.GamepadCmds):
        """Calculates and sets new joint angles from linear velocities.

        Args:
            cmd (GamepadCmds): Contains linear velocities for the arm.
        """
        vel = [cmd.arm_vx, cmd.arm_vy, cmd.arm_vz]

        ######################################################################
        # insert your code for finding "thetalist_dot"

        thetalist_dot = [0] * 5

        ######################################################################

        print(f"[DEBUG] Current thetalist (deg) = {self.joint_values}")
        print(
            f"[DEBUG] linear vel: {[round(vel[0], 3), round(vel[1], 3), round(vel[2], 3)]}"
        )
        print(f"[DEBUG] thetadot (deg/s) = {[round(td,2) for td in thetalist_dot]}")

        # Update joint angles
        dt = 0.5  # Fixed time step
        K = 1600  # mapping gain for individual joint control
        new_thetalist = [0.0] * 6

        # linear velocity control
        for i in range(5):
            new_thetalist[i] = self.joint_values[i] + dt * thetalist_dot[i]
        # individual joint control
        new_thetalist[0] += dt * K * cmd.arm_j1
        new_thetalist[1] += dt * K * cmd.arm_j2
        new_thetalist[2] += dt * K * cmd.arm_j3
        new_thetalist[3] += dt * K * cmd.arm_j4
        new_thetalist[4] += dt * K * cmd.arm_j5
        new_thetalist[5] = self.joint_values[5] + dt * K * cmd.arm_ee

        new_thetalist = [round(theta, 2) for theta in new_thetalist]
        print(f"[DEBUG] Commanded thetalist (deg) = {new_thetalist}")

        # set new joint angles
        self.set_joint_values(new_thetalist, radians=False)

    def DH_matrix(self, theta, d, r, alpha):
        """Calculates DH matrix based on given arguments"""
        return np.array(
            [
                [
                    cos(theta),
                    -sin(theta) * cos(alpha),
                    sin(theta) * sin(alpha),
                    r * cos(theta),
                ],
                [
                    sin(theta),
                    cos(theta) * cos(alpha),
                    -cos(theta) * sin(alpha),
                    r * sin(theta),
                ],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1],
            ]
        )

    def calc_DH_matrices(self, theta):
        """Calculates all DH Matrices of the system"""
        # DH table parameters
        theta_i_table = [
            theta[0],
            theta[1],
            theta[2],
            theta[3],
            theta[4],
        ]
        d_table = [self.l1, 0, 0, 0, self.l5]
        r_table = [0, self.l2, self.l3, self.l4, 0]
        alpha_table = [np.pi / 2, np.pi, np.pi, 0, 0]
        DH = np.zeros(shape=(5, 4, 4))
        for i in range(5):
            if i == 0:
                DH[i] = self.DH_matrix(
                    theta_i_table[i], d_table[i], r_table[i], alpha_table[i]
                ) @ self.DH_matrix(np.pi / 2, 0, 0, 0)
            elif i == 3:
                DH[i] = self.DH_matrix(
                    theta_i_table[i], d_table[i], r_table[i], alpha_table[i]
                ) @ self.DH_matrix(-np.pi / 2, 0, 0, -np.pi / 2)
            else:
                DH[i] = self.DH_matrix(
                    theta_i_table[i], d_table[i], r_table[i], alpha_table[i]
                )
        return DH

    def set_arm_position(self, x, y, z, rot):
        theta = [0, -85.04, -64.58, -69.54, 0, 0]

        theta[0] = atan2(y, x)
        print(theta[0])
        rot_z_theta1 = np.array(
            [
                [cos(theta[0]), -sin(theta[0]), 0],
                [sin(theta[0]), cos(theta[0]), 0],
                [0, 0, 1],
            ]
        )
        rotz = rot
        rot_y = np.array(
            [
                [cos(rotz), 0, sin(rotz)],
                [0, 1, 0],
                [-sin(rotz), 0, cos(rotz)],
            ]
        )
        k = np.transpose(np.array([[0, 0, 1]]))
        r_06 = rot_z_theta1 @ rot_y
        t_35 = (self.l4 + self.l5) * r_06 @ k

        p_wrist_x = x - t_35[0]
        p_wrist_y = y - t_35[1]
        p_wrist_z = z - t_35[2]

        rx = sqrt(p_wrist_x**2 + p_wrist_y**2)
        ry = p_wrist_z - self.l1

        theta[2] = -acos(
            (rx**2 + ry**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        )
        alpha = atan2(self.l2 * sin(theta[2]), self.l2 + self.l3 * cos(theta[2]))
        gamma = atan2(ry, rx)
        theta[1] = (gamma - alpha) - (np.pi / 2)
        theta[2] = -theta[2]

        DH = self.calc_DH_matrices(theta)
        r_03 = (DH[0] @ DH[1] @ DH[2])[:3, :3]
        r_35 = np.transpose(r_03) @ r_06

        theta[3] = atan2(r_35[0][0], r_35[0][2])

        theta = [degrees(i) for i in theta]

        return theta

    def set_joint_value(self, joint_id: int, theta: float, duration=250, radians=False):
        """Moves a single joint to a specified angle"""
        if not (1 <= joint_id <= 6):
            raise ValueError("Joint ID must be between 1 and 6.")

        if radians:
            theta = np.rad2deg(theta)

        theta = self.enforce_joint_limits(theta)
        positions = []
        for i in range(len(self.joint_values)):
            if i == joint_id - 1:
                positions.append([joint_id, self.angle_to_pulse(theta)])
            else:
                positions.append([i + 1, self.angle_to_pulse(self.joint_values[i])])
        self.board.bus_servo_set_position(1, positions)

        print(
            f"[DEBUG] Moving joint {joint_id} to {theta}° ({self.angle_to_pulse(theta)} pulse)"
        )
        time.sleep(self.joint_control_delay)

    def set_joint_values(self, thetalist: list, duration=1000, radians=False):
        """Moves all arm joints to the given angles.

        Args:
            thetalist (list): Target joint angles in degrees.
            duration (int): Movement duration in milliseconds.
        """
        if len(thetalist) != 6:
            raise ValueError("Provide 6 joint angles.")

        if radians:
            thetalist = [np.rad2deg(theta) for theta in thetalist]

        thetalist = self.enforce_joint_limits(thetalist)
        # self.joint_values = thetalist # updates joint_values with commanded thetalist
        thetalist = self.remap_joints(
            thetalist
        )  # remap the joint values from software to hardware

        positions = []
        for joint_id, theta in enumerate(thetalist, start=1):
            pulse = self.angle_to_pulse(theta)
            positions.append([joint_id, pulse])
        self.board.bus_servo_set_position(1, positions)

    def update_joint_value(self, joint_id: int):
        """Gets the joint angle"""
        count = 0
        while True:
            res = self.board.bus_servo_read_position(joint_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def update_joint_values(self):
        """Updates the joint angle values by calling "get_joint_value" for all joints"""
        res = [self.update_joint_value(i + 1) for i in range(len(self.joint_values))]
        res = self.remap_joints(res)

        # check for Nones and replace with the previous value
        for i in range(len(res)):
            if res[i] is None:
                res[i] = self.joint_values[i]
            else:
                res[i] = self.pulse_to_angle(res[i][0])
        self.joint_values = res

    def get_joint_value(self, joint_id: int):
        """Gets the joint angle"""
        return self.joint_values[joint_id]

    def get_joint_values(self):
        """Returns all the joint angle values"""
        return self.joint_values

    def enforce_joint_limits(self, thetalist: list) -> list:
        """Clamps joint angles within their hardware limits.

        Args:
            thetalist (list): List of target angles.

        Returns:
            list: Joint angles within allowable ranges.
        """
        return [
            np.clip(theta, *limit) for theta, limit in zip(thetalist, self.joint_limits)
        ]

    def move_to_home_position(self):
        self.board.set_buzzer(2400, 0.1, 0.9, 1)
        time.sleep(2)

        print(f"Moving to home position...")
        self.set_joint_values(self.home_position, duration=1000)
        time.sleep(2.0)
        print(f"Arrived at home position: {self.joint_values} \n")
        time.sleep(1.0)
        print(f"------------------- System is now ready!------------------- \n")

    def move_to_position_1(self):
        """
        Move in square motion
        """
        print(f"Moving to position 1...")
        self.set_joint_values(
            self.set_arm_position(0.1866, 0.1155, 0.3671, 1.2012),
            duration=500,
        )
        time.sleep(2.0)
        print(f"Arrived at position 1: {self.joint_values} \n")
        time.sleep(1.0)
        print(f"------------------- System is now ready!------------------- \n")

    # -------------------------------------------------------------
    # Utility Functions
    # -------------------------------------------------------------

    def angle_to_pulse(self, x: float):
        """Converts degrees to servo pulse value"""
        hw_min, hw_max = 0, 1000  # Hardware-defined range
        joint_min, joint_max = -150, 150
        return int(
            (x - joint_min) * (hw_max - hw_min) / (joint_max - joint_min) + hw_min
        )

    def pulse_to_angle(self, x: float):
        """Converts servo pulse value to degrees"""
        hw_min, hw_max = 0, 1000  # Hardware-defined range
        joint_min, joint_max = -150, 150
        return round(
            (x - hw_min) * (joint_max - joint_min) / (hw_max - hw_min) + joint_min, 2
        )

    def stop_motors(self):
        """Stops all motors safely"""
        self.board.set_motor_speed([0] * 4)
        print("[INFO] Motors stopped.")

    def remap_joints(self, thetalist: list):
        """Reorders angles to match hardware configuration.

        Args:
            thetalist (list): Software joint order.

        Returns:
            list: Hardware-mapped joint angles.

        Note: Joint mapping for hardware
            HARDWARE - SOFTWARE
            joint[0] = gripper/EE
            joint[1] = joint[5]
            joint[2] = joint[4]
            joint[3] = joint[3]
            joint[4] = joint[2]
            joint[5] = joint[1]
        """
        return thetalist[::-1]
