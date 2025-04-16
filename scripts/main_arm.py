import argparse
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from hiwonder import HiwonderRobot
import math
from utils import wraptopi, EndEffector
from trajectory_generator import MultiAxisTrajectoryGenerator
import time
import yaml


class Visualizer:
    """
    A class for visualizing and controlling a robot manipulator, including forward and inverse kinematics, 
    and velocity control through a Tkinter GUI.
    """

    def __init__(self, root, args):
       """
       Nothing rn
       """
        


    def reset_joints(self):
        """
        Resets all joint angles to 0 and updates the forward kinematics.
        """
        theta = [0.0] * self.robot.num_joints
        self.robot.reset_ee_trajectory()
        self.update_FK(theta)


    def get_ee_from_input(self):
        EE = EndEffector()
        EE.x = float(self.pose_button[0].get())
        EE.y = float(self.pose_button[1].get())
        EE.z = float(self.pose_button[2].get())
        EE.rotx = float(self.pose_button[3].get())
        EE.roty = float(self.pose_button[4].get())
        EE.rotz = float(self.pose_button[5].get())
        return EE
    

    def solve_IK1(self):
        """
        Solves the inverse kinematics for a given end-effector pose using the first solution.
        """
        self.update_IK(pose=self.get_ee_from_input(), soln=0)


    def solve_IK2(self):
        """
        Solves the inverse kinematics for a given end-effector pose using the second solution.
        """
        self.update_IK(pose=self.get_ee_from_input(), soln=1)


    def numerical_solve(self):
        """
        Solves the inverse kinematics for a given end-effector pose using a numerical method.
        """
        self.update_IK(pose=self.get_ee_from_input(), soln=1, numerical=True)


    def update_FK(self, theta: list, display_traj=False):
        """
        Updates the forward kinematics plot based on the given joint angles.

        Args:
            theta (list): List of joint angles.
        """
        if display_traj:
            self.HiwonderRobot.update_ee_trajectory()
        
        try:
            self.robot.update_plot(angles=theta)
            self.canvas.draw()
            self.canvas.flush_events()
        except ValueError:
            tk.messagebox.showerror("Input Error", "Please enter valid numbers")


    def update_IK(self, pose: EndEffector, soln=0, numerical=False, display_traj=False):
        """
        Updates the inverse kinematics plot based on the given end-effector pose.

        Args:
            pose (EndEffector): The desired end-effector pose.
            soln (int, optional): The solution index to use. Defaults to 0.
            numerical (bool, optional): Whether to use a numerical solver. Defaults to False.
        """
        if display_traj:
            self.robot.update_ee_trajectory()
        
        if numerical:
            self.robot.update_plot(pose=pose, soln=soln, numerical=False)
        else:
            self.robot.update_plot(pose=pose, soln=soln)
        self.canvas.draw()
        self.canvas.flush_events()


    def solve_IK(self, pose: EndEffector, soln=0):
        return self.robot.solve_inverse_kinematics(pose, soln=soln)



    def update_waypoints(self):
        """
        Loads waypoints from a YAML file and updates the robot's internal waypoint list and plot.
        """

        print('Updating waypoints...')

        # get pid_gains from yaml file
        with open('waypoints.yml', 'r') as file:
            waypoints = yaml.safe_load(file)

        self.waypoint_idx = 0
        self.robot.update_waypoints(waypoints['points'])
        self.robot.plot_3D()
        self.canvas.draw()

    
    def generate_traj_task_space(self):
        
        """
        Generates and visualizes a task-space trajectory using a polynomial interpolator between waypoints.
        """
    
        print('Following trajectory in task space...')
    
        waypoints = self.robot.get_waypoints()
        q0 = waypoints[0]
        qf = waypoints[1]

        traj = MultiAxisTrajectoryGenerator(method="cubic", mode="task", interval=[0, 1], ndof=len(q0), start_pos=q0, final_pos=qf)
        traj_dofs = traj.generate(nsteps=50)

        for i in range(50):
            pos = [dof[0][i] for dof in traj_dofs]
            ee = EndEffector(*pos, 0, -math.pi/2, wraptopi(math.atan2(pos[1], pos[0]) + math.pi))
            self.update_IK(ee, soln=0, numerical=True, display_traj=True)
            time.sleep(0.05)
        
        traj.plot()

    
    def generate_traj_joint_space(self):
        """
        Generates and visualizes a joint-space trajectory by solving inverse kinematics at waypoints
        and interpolating between resulting joint configurations.
        """

        print('Following trajectory in joint space...')
        
        waypoints = self.robot.get_waypoints()

        EE_0 = EndEffector(*waypoints[0], 0, 0, 0)
        EE_f = EndEffector(*waypoints[1], 0, 0, 0)

        q0 = np.rad2deg(HiwonderRobot.set_arm_position(EE_0))
        qf = np.rad2deg(HiwonderRobot.set_arm_position(EE_f))

        traj = MultiAxisTrajectoryGenerator(method="cubic", mode="joint", interval=[0, 1], ndof=len(q0), start_pos=q0, final_pos=qf)

        traj_dofs = traj.generate(nsteps=50)

        for i in range(50):
            theta = [dof[0][i] for dof in traj_dofs]            
            self.update_FK(theta=theta, display_traj=True) 
            time.sleep(0.05)
        
        traj.plot()
