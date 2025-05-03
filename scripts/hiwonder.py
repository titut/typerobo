# hiwonder.py
"""
Hiwonder Robot Controller
-------------------------
Handles the control of the mobile base and 5-DOF robotic arm using commands received from the gamepad.
"""

import time
from math import sin, cos, atan2, radians, degrees, sqrt, acos
import math
from utils import wraptopi, EndEffector
import numpy as np
import csv
from ros_robot_controller_sdk import Board
from bus_servo_control import *
from trajectory_generator import MultiAxisTrajectoryGenerator

import utils as ut

# Robot base constants
WHEEL_RADIUS = 0.047  # meters
BASE_LENGTH_X = 0.096  # meters
BASE_LENGTH_Y = 0.105  # meters


class HiwonderRobot:
    def __init__(self):
        """
        Initialize motor controllers, servo bus, and default robot states and variables.
        """
        self.board = Board()
        self.board.enable_reception()
        self.bsc = BusServoControl(self.board)

        # lengths of arm
        self.l1, self.l2, self.l3, self.l4, self.l5 = 0.155, 0.099, 0.095, 0.055, 0.105
        self.cam_offset = 0.045
        self.cam_DH = self.DH_matrix(np.pi, 0, self.cam_offset, 0) @ self.DH_matrix(
            np.pi / 2, 0, 0, 0
        )

        # current joint_values
        self.joint_values = [0, 10, 120, -90, 0, 0]  # degrees

        # home position - looking down at the ground
        self.home_position = [0, 10, 120, -90, 0, 0]  # degrees

        # joint limits
        self.joint_limits = [
            [-120, 120],
            [-90, 90],
            [-120, 120],
            [-100, 100],
            [-90, 90],
            [-120, 30],
        ]
        self.theta_limits = [
            [-np.pi, np.pi],
            [-np.pi / 3, np.pi / 3],
            [-np.pi + np.pi / 12, np.pi - np.pi / 12],
            [-np.pi + np.pi / 12, np.pi - np.pi / 12],
            [-np.pi, np.pi],
        ]

        # maximum timout
        self.time_out = 100

        self.move_to_home_position()

    def get_servo_pos(self):
        current_angle = []
        for i in range(6):
            current_angle.append(self.bsc.getBusServoPulse(i+1)[0])
        return current_angle

    def generate_traj_task_space(self):
        """
        Generates and visualizes a task-space trajectory using a polynomial interpolator between waypoints.
        """

        print("Generating trajectory in task space...")
        # if the desired z value is below 0.1, move first to the location
        # (x, y, 0.1), then move arm vertically down to (x, y, z)
        two_part_soln = False

        # solve forward kinematics of current angle
        q = [radians(i) for i in self.joint_values]
        q0 = self.solve_forward_kinematics(q)[0:3]
        qf = self.test_pos

        if qf[2] < 0.1:
            two_part_soln = True

        # generate trajectory in task-space
        traj = MultiAxisTrajectoryGenerator(
            method="quintic",
            mode="task",
            interval=[0, 1],
            ndof=len(q0),
            start_pos=q0,
            final_pos=(qf if not two_part_soln else [qf[0], qf[1], 0.1]),
        )
        steps = 25
        traj_dofs = traj.generate(nsteps=steps)

        # list of theta values to go to
        path_theta_list = []

        # Convert task-space positions to joint-space
        for i in range(50):
            pos = [dof[0][i] for dof in traj_dofs]
            ee = EndEffector(
                *pos,
                0,
                -math.pi / 2,
                wraptopi(math.atan2(pos[1], pos[0]) + math.pi),
            )
            path_theta_list.append(self.set_arm_position(ee.x, ee.y, ee.z))

        print("Trajectory generated, starting movement...")

        # move!
        for i in path_theta_list:
            move_time = 0.15
            self.set_joint_values(i, move_time)
            time.sleep(move_time*1.1)

        print(f"Arrived at desired location: {qf}")

        print("\n\n")

    def set_robot_commands(self, cmd: ut.GamepadCmds):
        """Updates robot base and arm based on gamepad commands.

        Args:
            cmd (GamepadCmds): Command data class with velocities and joint commands.
        """

        test_x = input("x: ")
        test_y = input("y: ")
        test_z = input("z: ")
        if test_z == "home":
            self.move_to_home_position()
        else:
            self.test_pos = [float(test_x), float(test_y), float(test_z)]
            self.generate_traj_task_space()
            print("\n\n")

    def solve_forward_kinematics(self, theta):
        """
        Given a list of thetas (in radians) calculate its expected position
        in xyz coordinates.

        Args:
            theta: list of joint-angles in radians
        """
        EE = np.array([0, 0, 0, 1])

        DH = self.calc_DH_matrices(theta)
        T_cumulative = [np.eye(4)]
        for fk_i in range(5):
            T_cumulative.append(T_cumulative[-1] @ DH[fk_i])

        EE = T_cumulative[5] @ EE
        return EE

    def DH_matrix(self, theta, d, r, alpha):
        """
        Calculates DH matrix based on given DH parameters

        Args:
            theta: rotation around z-axis
            d: distance in z-axis
            r: distance in x-axis
            alpha: rotation around x-axis
        """
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
        """
        Calculates all DH Matrices of the system

        Args:
            theta: list of joint values in radians
        """
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

        # Calculate all DH matrices
        for dh_i in range(5):
            if dh_i == 0:
                DH[dh_i] = self.DH_matrix(
                    theta_i_table[dh_i], d_table[dh_i], r_table[dh_i], alpha_table[dh_i]
                ) @ self.DH_matrix(np.pi / 2, 0, 0, 0)
            elif dh_i == 3:
                DH[dh_i] = self.DH_matrix(
                    theta_i_table[dh_i], d_table[dh_i], r_table[dh_i], alpha_table[dh_i]
                ) @ self.DH_matrix(-np.pi / 2, 0, 0, -np.pi / 2)
            else:
                DH[dh_i] = self.DH_matrix(
                    theta_i_table[dh_i], d_table[dh_i], r_table[dh_i], alpha_table[dh_i]
                )
        return DH

    def jacobian(self, theta):
        """
        Calculate the Jacobian given a list of thetas

        Args:
            theta: list of joint angles in radians
        """
        DH = self.calc_DH_matrices(theta)

        T_cumulative = [np.eye(4)]
        for i in range(5):
            T_cumulative.append(T_cumulative[-1] @ DH[i])

        # Define O0 for calculations
        O0 = np.array([0, 0, 0, 1])

        # Initialize the Jacobian matrix
        jacobian = np.zeros((3, 5))

        # Calculate the Jacobian columns
        for i in range(5):
            T_curr = T_cumulative[i]
            T_final = T_cumulative[-1]

            # Calculate position vector r
            r = (T_final @ O0 - T_curr @ O0)[:3]

            # Compute the rotation axis z
            z = T_curr[:3, :3] @ np.array([0, 0, 1])

            # Compute linear velocity part of the Jacobian
            jacobian[:, i] = np.cross(z, r)

        return np.where(np.isclose(jacobian, 0, atol=1e-5), 0, jacobian)

    def damped_inverse_jacobian(self, q=None, damping_factor=0.025):
        """
        Calculate the damped inverse jacobian given a list of thetas.

        Args:
            q: list of joint angles in radians
            damping_factor: float
        """
        J = self.jacobian(q)
        JT = np.transpose(J)
        I = np.eye(3)
        return JT @ np.linalg.inv(J @ JT + (damping_factor**2) * I)

    def set_arm_position(self, x, y, z, tol=1e-3, ilimit=500):
        """
        Calculate numerical inverse kinematics based on input coordinates.

        Args:
            x: float
            y: float
            z: float
            tol (float): acceptable error in resulting ik
            ilimit (int): max number of iterations
        """

        Te_d = [x, y, z]

        # Iteration count
        i = 0
        q = [radians(i) for i in self.joint_values]
        q = q[:-1]

        while i < ilimit:
            i += 1

            # compute current EE position based on q
            Te = self.solve_forward_kinematics(q)

            # calculate the EE position error
            e = [0, 0, 0]
            e[0] = Te_d[0] - Te[0]
            e[1] = Te_d[1] - Te[1]
            e[2] = Te_d[2] - Te[2]

            # update q
            q += self.damped_inverse_jacobian(q) @ e

            # check for joint limits
            for j, th in enumerate(q):
                q[j] = np.clip(th, self.theta_limits[j][0], self.theta_limits[j][1])

            # Check if we have arrived
            if abs(max(e, key=abs)) < tol:
                break

        if abs(max(e, key=abs)) > tol:
            print(
                "\n [ERROR] Numerical IK solution failed to converge... \n \
                  Possible causes: \n \
                  1. cartesian position is not reachable by the robot, given the joint limits \n \
                  2. desired joint configuration is very close to OR at a singularity \n \
                  3. iterative algorithm is stuck at a local minima \n \
                  4. solver is taking too long to converge  \n"
            )
            print(
                f"Max position error: {max(e, key=abs)} | # iterations: {i}/{ilimit} "
            )
            # raise ValueError
            return False

        theta = [degrees(i) for i in q]
        theta.append(0)

        print(
            f"Solution found = {theta} | Max pos error = {max(e, key=abs)} | # iterations: {i}/{ilimit}  \n"
        )

        # q = [radians(i) for i in theta]
        # print("Target Pos:")
        # # print(self.solve_forward_kinematics(q)[0:3])
        # print(theta)
        return theta

    def pose_cam2world_frame(self, x, y, z):
        """
        Given x, y, and z in the camera frame, return the respective pose
        in the world frame

        Args:
            x: float
            y: float
            z: float
        """
        theta = [radians(i) for i in self.joint_values]
        DH = self.calc_DH_matrices(theta)

        # calculate cumulative transformation matrices
        T_cumulative = [np.eye(4)]
        for i in range(5):
            T_cumulative.append(T_cumulative[-1] @ DH[i])

        # convert cam frame to world frame
        pose_cam_frame = np.array([x, y, z, 1])
        pose_world_frame = T_cumulative[4] @ self.cam_DH @ pose_cam_frame

        return pose_world_frame[:3]

    def set_joint_values(self, thetalist: list, duration=1, radians=False):
        """Moves all arm joints to the given angles.

        Args:
            thetalist (list): Target joint angles in degrees.
            duration (int): Movement duration in milliseconds.
        """
        if len(thetalist) != 6:
            raise ValueError("Provide 6 joint angles.")

        thetalist_real = [11 * theta / 9 for theta in thetalist]

        thetalist_real = self.enforce_joint_limits(thetalist_real)
        self.joint_values = thetalist  # updates joint_values with commanded thetalist
        thetalist_real = self.remap_joints(
            thetalist_real
        )  # remap the joint values from software to hardware

        # print(f"{self.joint_values=}")

        positions = []
        for joint_id, theta in enumerate(thetalist_real, start=1):
            pulse = self.angle_to_pulse(theta)
            positions.append([joint_id, pulse])
        self.board.bus_servo_set_position(duration, positions)

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
        """
        Move the arm to home position
        """
        time.sleep(2)
        print("Moving to home position...")
        self.set_joint_values(self.home_position, duration=1000)
        time.sleep(2.0)
        print(f"Arrived at home position: {self.joint_values} \n")
        time.sleep(1.0)
        print("------------------- System is now ready!------------------- \n")

    # -------------------------------------------------------------
    # Utility Functions
    # -------------------------------------------------------------

    def angle_to_pulse(self, x: float):
        """
        Converts degrees to servo pulse value

        Args:
            x (float): angle of joint in degrees
        """
        hw_min, hw_max = 0, 1000  # Hardware-defined range
        joint_min, joint_max = -150, 150
        return int(
            (x - joint_min) * (hw_max - hw_min) / (joint_max - joint_min) + hw_min
        )

    def pulse_to_angle(self, x: float):
        """
        Converts servo pulse value to degrees

        Args:
            x (float): servo pulse of joint
        """
        hw_min, hw_max = 0, 1000  # Hardware-defined range
        joint_min, joint_max = -150, 150
        return round(
            (x - hw_min) * (joint_max - joint_min) / (hw_max - hw_min) + joint_min, 2
        )

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
