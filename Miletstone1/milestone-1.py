import sys
sys.path.append('../config')
import numpy as np

class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7
    
    def forward_kinematcis(self, dh_parameters, thetas):
        """
        Compute foward kinematics
        
        Your implementation should:
        1. Compute transformation matrices for each frame using DH parameters
        2. Compute end-effector pose
        
        Parameters
        ----------
        dh_parameters: np.ndarray
            DH parameters (you can choose to apply the offset to the tool flange, center of gripper, or the pen tip)
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            End-effector pose
        """
        if thetas.ndim != 1:
            raise ValueError('Expecting a 1D array of joint angles.')

        if thetas.shape[0] != self.dof:
            raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}')
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO
        
        T = np.eye(4) 

        for i in range(len(thetas)):
            linkLength, linkTwist, linkOffset, jointAngleOffset = dh_parameters[i]

            jointAngle = thetas[i] + jointAngleOffset # getting offset 

            cosTheta, sinTheta = np.cos(jointAngle), np.sin(jointAngle) # getting angles
            cosAlpha, sinAlpha = np.cos(linkTwist), np.sin(linkTwist)

            # DH convention for i to i+1
            HCurrentToNext = np.array([
                [cosTheta, -sinTheta * cosAlpha,  sinTheta * sinAlpha, linkLength * cosTheta],
                [sinTheta,  cosTheta * cosAlpha, -cosTheta * sinAlpha, linkLength * sinTheta],
                [0.0,       sinAlpha,             cosAlpha,             linkOffset],
                [0.0,       0.0,                  0.0,                  1.0]
            ])

            T = T @ HCurrentToNext

        return T

        
        # --------------- END STUDENT SECTION --------------------------------------------------
    
    