import math

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

# The link class, including the DH parameters, transformation matrix and basic Jacobian matrix
class Link:
    # Initialize the class with dh_parameters
    def __init__(self, dh_params):
        self.dh_params_ = dh_params
    
    # Define the transformation matrix T_i^(i-1) for the i-th link
    def transformation_matrix(self, theta):
        alpha = self.dh_params_[0]
        a = self.dh_params_[1]
        d = self.dh_params_[2]
        theta = theta+self.dh_params_[3]
        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)

        # Calculate the transform matrix of the i-th link to (i-1)-th link
        trans = np.array([[ct, -st, 0, a],
                          [st*ca, ct * ca, - sa, -sa * d],
                          [st*sa, ct * sa,   ca,  ca * d],
                          [0, 0, 0, 1]])
        return trans

    # Define the basic Jacobian matrix for the i-th link
    @staticmethod
    def basic_jacobian(trans, ee_pos):

        pos = np.array(
            [trans[0, 3], trans[1, 3], trans[2, 3]])
        z_axis = np.array(
            [trans[0, 2], trans[1, 2], trans[2, 2]])

        basic_jacobian = np.hstack(
            (np.cross(z_axis, ee_pos - pos), z_axis))
        return basic_jacobian


# The NLinkArm class, including the forward kinematics, inverse kinematics and basic Jacobian matrix
class NLinkArm:

    # Instantiate the class with a list of Links
    def __init__(self, dh_params_list) -> None:
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))

    # Calculate the transform matrix from the end effector to the base
    def transformation_matrix(self, thetas):
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(
                trans, self.link_list[i].transformation_matrix(thetas[i]))
        return trans

    # Calculate the forward kinematics
    def forward_kinematics(self, thetas):
        trans = self.transformation_matrix(thetas)
        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        
        alpha, beta, gamma = self.euler_angle(thetas)
        return [x, y, z, alpha, beta, gamma]

    # Calculate the rotational angles of the end effector
    def euler_angle(self, thetas):
        trans = self.transformation_matrix(thetas)

        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi
        beta = math.atan2(
            trans[0][2] * math.cos(alpha) + trans[1][2] * math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0] * math.sin(alpha) + trans[1][0] * math.cos(alpha),
            -trans[0][1] * math.sin(alpha) + trans[1][1] * math.cos(alpha))

        return np.degrees(alpha), np.degrees(beta), np.degrees(gamma)


    def error_function(self, thetas, ref_ee_pose):
        # Calculate the error between current end-effector pose
        # and reference end-effector pose using forward kinematics
        ee_pose = self.forward_kinematics(thetas)
        ee_pose = np.array(ee_pose)[0:3]
        error = np.linalg.norm(ref_ee_pose - ee_pose)
        return error

    def inverse_kinematics(self, ref_ee_pose, initial_guess):
        # Define the optimization bounds for joint angles
        base_bounds = [(-np.radians(154), np.radians(154)),
                    (-np.radians(150), np.radians(150)),
                    (-np.radians(150), np.radians(150)),
                    (-np.radians(149), np.radians(149)),
                    (-np.radians(145), np.radians(145)),
                    (-np.radians(149), np.radians(149))]

        # Define the optimization problem
        result = minimize(self.error_function, initial_guess, args=(ref_ee_pose,),
                        bounds=base_bounds, method='SLSQP')
        
        # Return the solution with the smallest error
        return result.x
    

    def basic_jacobian(self, thetas):
        ee_pos = self.forward_kinematics(thetas)[0:3]
        basic_jacobian_mat = []
        # Initialize the transformation matrix
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            # Calculate the transformation matrix of each joint
            trans = np.dot(trans,self.link_list[i].transformation_matrix(thetas[i])) 

            # Calculate the basic Jacobian matrix of each joint
            basic_jacobian_mat.append(
                self.link_list[i].basic_jacobian(trans, ee_pos))  
        return np.array(basic_jacobian_mat).T
    
class Gen3LiteArm:
    def __init__(self):
        # Define DH Matrix
        self.dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                [math.pi, 280/1000, 0, 0+math.pi/2],
                                [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                [math.pi/2, 0, 57/1000, 0],
                                [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        # Instantiate the gen3_lite robot
        self.arm = NLinkArm(self.dh_params_list)

if __name__=="__main__":
    arm_model=Gen3LiteArm()
    # Test the forward kinematics
    home=np.array([0,345,75,0,300,0])
    home=np.radians(home)
    print(arm_model.arm.forward_kinematics(home))