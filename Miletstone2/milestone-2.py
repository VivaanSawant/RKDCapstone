import numpy as np
import matplotlib.pyplot as plt
from Miletstone2.FrankaRobot16384 import Franka16384

class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7

        # Example DH parameters for Franka Emika 7-DOF robot
        # TODO: Fill in with actual DH parameters copy from milestone 1
        self.dh_parameters = ...  # Fill in with actual DH parameters

        # for acceleration, deceleration phases
        #TODO: You may adjust these parameters as needed
        self.total_time = 2.0  # total time for the trajectory
        self.accel_time = 0.5  # seconds
        self.decel_time = 0.5  # seconds
    
    def forward_kinematics(self, dh_parameters, thetas):
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
        dh_parameters = np.array([
            [0.0,       0.0,        0.333,   0.0],
            [0.0,      -np.pi/2,    0.0,     0.0],
            [0.0,       np.pi/2,    0.316,   0.0],
            [0.0825,   np.pi/2,    0.0,     0.0],
            [-0.0825,   -np.pi/2,    0.384,   0.0],
            [0.0,       np.pi/2,    0.0,     0.0],
            [0.088,     np.pi/2,    0.2104, -np.pi/4]
        ])
        
        T = np.eye(4) 

        for i in range(len(thetas)):
            if(len(dh_parameters)<=0):
                raise ValueError(f'Invalid size of dh_parameters: {len(dh_parameters)}')
            if(len(thetas)<=0):
                raise ValueError(f'Invalid size of thetas: {len(thetas)}')
            a, alpha, d, theta_offset = dh_parameters[i] # getting params

            theta = thetas[i] + theta_offset # getting offset

            cosTheta, sinTheta = np.cos(theta), np.sin(theta) #angles
            cosAlpha, sinAlpha = np.cos(alpha), np.sin(alpha) #angles

            # modivied form
            HCurrentToNext = np.array([
                [cosTheta, -sinTheta, 0.0, a],
                [sinTheta * cosAlpha, cosTheta * cosAlpha, -sinAlpha, -d * sinAlpha],
                [sinTheta * sinAlpha, cosTheta * sinAlpha,  cosAlpha,  d * cosAlpha],
                [0.0, 0.0, 0.0, 1.0]
            ])
            

            T = T @ HCurrentToNext

        return T

    # ------------------------------------------------------------------------------------------
    # STUDENT EXTENSION: Linear Interpolation Trajectory (Lerp) and Trapezoidal Velocity Profile
    # ----------------------------------------------------------------------------------------
    def compute_joint_trajectory(self, q0, q1, num_steps):
        """
        Compute an interpolation-based joint-space trajectory between q0 and q1.

        Your implementation should:
        1. Interpolate between q0 and q1 to generate intermediate waypoints
        2. apply to achieve trapezoidal velocity or other smooth motion profiles

        Parameters
        ----------
        q0, q1 : np.ndarray
            Start and goal joint configurations (7x1 each)
        num_steps : int
            Number of waypoints to generate
        
        Returns
        -------
        np.ndarray
            Trajectory as an array of shape (N, 7) containing interpolated
            joint angles, where N is the number of waypoints.
        
        Notes
        -----
        - You may choose:
            * total_time (e.g., 2.0 s)
            * number of waypoints (e.g., 20)
            * acceleration and deceleration durations (e.g., 0.5 s each)
        - Start simple: use linear interpolation (LERP).
        - Then extend it with trapezoidal.
        """
        q0, q1 = np.array(q0), np.array(q1)
        if q0.shape[0] != self.dof or q1.shape[0] != self.dof:
            raise ValueError(f"Expecting joint vectors of length {self.dof}")


        

        # ---------------- BEGIN STUDENT SECTION ----------------
        # TODO: Implement interpolation here
        # 
        # You are welcome to try any other interpolation methods as well. 
        # Belkow is just a simple example to get you started.


        # -------------------------------------------------------
        # Example (simple linear interpolation):
        # N = 20
        # t = np.linspace(0, 1, N)
        # traj = np.outer(t, q1 - q0) + q0
        #
        
        displacement = q1 - q0
        # To make smoother motion, implement trapezoidal velocity profile:
        T =  self.total_time
        accel_time = self.accel_time
        decel_time = self.decel_time
        const_time = T - accel_time - decel_time
        
        time_samples = np.linspace(0, T, num_steps)
        
        traj = np.zeros((num_steps, 7)) # n, 7 shape
        
        scaling_factor = 1.0 / (const_time + 0.5 * accel_time + 0.5 * decel_time)
        
        for i, t in enumerate(time_samples):
            
            interp = 0.0
            
            if(t <= accel_time):
                interp = 0.5 * scaling_factor * (t**2 / accel_time)
                #accelerating
            elif(t <= accel_time + const_time):
                #const velocity
                interp = (0.5 * scaling_factor * accel_time) + scaling_factor * (t - accel_time)
            else:
                #d ecelerating
                decline_time = t - (accel_time + const_time)
                interp = (0.5 * scaling_factor * accel_time) + scaling_factor * const_time + scaling_factor * decline_time - 0.5 * scaling_factor * (decline_time**2 / decel_time)
                
            traj[i, :] = q0 + interp * displacement
        
        #
        #
        return traj
        # -------------------------------------------------------
        


    def plot_end_effector_trajectory(self, traj):
        """
        Plot the end-effector trajectory for a given joint-space trajectory
        using the robot's DH parameters stored in self.dh_parameters.
        
        Parameters
        ----------
        traj : np.ndarray
            Array of joint configurations, shape (N, 7)
        """

        ee_positions = []

        #TODO: Compute end-effector positions along the trajectory
        for q in traj:
            #print(len(traj))
            #q = traj[i]
            ee_pose = self.forward_kinematics(self.dh_parameters, q) # Compute FK using self.forward_kinematics
            ee_positions.append(ee_pose[:3, 3])  # Extract XYZ position

        ee_positions = np.array(ee_positions)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2], marker='o')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('End-Effector Trajectory')
        plt.savefig('end_effector_trajectory.png')
        plt.show()



    # ---------------------------------------------------------------
    # TODO: Compute the Jacobian analytically
    # ---------------------------------------------------------------
    def compute_jacobian_analytical(self, thetas):
        """
        Compute the analytical Jacobian for the given joint configuration.

        Parameters
        ----------
        thetas : np.ndarray
            Joint configuration (7,)

        Returns
        -------
        J : np.ndarray
            6x7 Jacobian matrix [linear; angular]
        """
        if thetas.ndim != 1:
            raise ValueError("Expecting a 1D array of joint angles.")
        if thetas.shape[0] != self.dof:
            raise ValueError(f"Expected {self.dof} joint angles, got {thetas.shape[0]}")

        # ---------------- BEGIN STUDENT SECTION ----------------
        # TODO: Implement the analytical Jacobian computation
        #
        # Example:
        # J = np.zeros((6, self.dof))
        # for i in range(self.dof):
        #     ...
        # return J
        # --------------------------------------------------------
        dh_parameters = np.array([
            [0.0,       0.0,        0.333,   0.0],
            [0.0,      -np.pi/2,    0.0,     0.0],
            [0.0,       np.pi/2,    0.316,   0.0],
            [0.0825,   np.pi/2,    0.0,     0.0],
            [-0.0825,   -np.pi/2,    0.384,   0.0],
            [0.0,       np.pi/2,    0.0,     0.0],
            [0.088,     np.pi/2,    0.2104, -np.pi/4]
        ])
        J = np.zeros((6, self.dof))
        T = np.eye(4)
        Ts = [T.copy()]            
        for i in range(len(thetas)):
            if(len(dh_parameters)<=0):
                raise ValueError(f'Invalid size of dh_parameters: {len(dh_parameters)}')
            if(len(thetas)<=0):
                raise ValueError(f'Invalid size of thetas: {len(thetas)}')
            a, alpha, d, theta_offset = dh_parameters[i] # getting params

            theta = thetas[i] + theta_offset # getting offset

            cosTheta, sinTheta = np.cos(theta), np.sin(theta) #angles
            cosAlpha, sinAlpha = np.cos(alpha), np.sin(alpha) #angles

            # modivied form
            H_i = np.array([
                [cosTheta, -sinTheta, 0.0, a],
                [sinTheta * cosAlpha, cosTheta * cosAlpha, -sinAlpha, -d * sinAlpha],
                [sinTheta * sinAlpha, cosTheta * sinAlpha,  cosAlpha,  d * cosAlpha],
                [0.0, 0.0, 0.0, 1.0]
            ])
            

            T = T @ H_i
            Ts.append(T.copy())

        origins = []
        z_axes = []
        for T_i in Ts:
            origins.append(T_i[0:3, 3])
            z_axes.append(T_i[0:3, 2])
            

        o_n = origins[-1]
        for i in range(self.dof):
            J[0:3, i] = np.cross(z_axes[i], o_n - origins[i])
            J[3:6, i] = z_axes[i]

        return J

    # ---------------------------------------------------------------
    # TODO: Compute the Jacobian numerically
    # ---------------------------------------------------------------
    def compute_jacobian_numerical(self, thetas, delta=1e-4):
        """
        Compute the numerical Jacobian by finite differences.

        Parameters
        ----------
        thetas : np.ndarray
            Joint configuration (7,)
        delta : float
            Small change for numerical differentiation

        Returns
        -------
        J : np.ndarray
            6x7 Jacobian matrix [linear; angular]
        """
        if thetas.ndim != 1:
            raise ValueError("Expecting a 1D array of joint angles.")
        if thetas.shape[0] != self.dof:
            raise ValueError(f"Expected {self.dof} joint angles, got {thetas.shape[0]}")

        # Compute base pose
        T0 = self.forward_kinematics(thetas)
        pos0 = T0[:3, 3]
        R0 = T0[:3, :3]

        J = np.zeros((6, self.dof))

        # ---------------- BEGIN STUDENT SECTION ----------------
        # TODO: Implement numerical differentiation
        # For each joint i:
        #   perturb theta_i by +delta
        #   compute new FK
        #   compute difference in position and orientation
        #   fill J columns as (Δx/δ, Δθ/δ)
        #
        # Hint: To get Δθ, use rotation difference:
        #       R_err = R0.T @ R1
        #       rot_vec = np.arctan2(..., ...)   # or convert to axis-angle
        #
        # Return the concatenated Jacobian.
        # --------------------------------------------------------
        for i in range (self.dof):
            thetasPerturbed = thetas.copy()
            thetasPerturbed[i] += delta

            T1 = self.forward_kinematics(thetasPerturbed)
            pos1 = T1[:3, 3]
            R1 = T1[:3, :3]
            Pdifference = (pos1 - pos0) / delta # postion

            Rerror = R0.T @ R1 # rototaion
            dtheta = np.array([
                Rerror[2, 1] - Rerror[1, 2],
                Rerror[0, 2] - Rerror[2, 0],
                Rerror[1, 0] - Rerror[0, 1]
            ]) / (2 * delta)

            J[0:3, i] = Pdifference
            J[3:6, i] = dtheta
        return J
        #raise NotImplementedError("Implement compute_jacobian_numerical")
def main():
    traj = np.array([[0.02204015, -0.35898457,  0.02613999, -2.364214, 0.00274931,  2.04544125, 0.82331108],
    [-1.41190431, -1.12662768,  1.05950991, -2.3611671,0.92804899,  1.57607987,  0.0403548 ],
    [-0.07447891, -0.03003863, -0.18793599, -2.47069798, 0.09206394, 2.57397382, 0.39425526]
    ])
    
    q0 = np.array([1.55361583e-02, 1.06624155e-01, 2.04122546e-02, 
                            -2.00029706e+00, -3.49519799e-04,  2.06950945e+00,  8.05155261e-01])
    q1 = np.array([ 0.17939389, -0.09461021,  0.13338035, -1.82398707,  0.12518016,
                            1.6630852,1.07733918])
    q2 = np.array([0.17105211,  0.01692119, -0.53377664, -1.89941006,  0.07431345,
                            1.66426027,0.32460998])
    q3 = np.array([ 1.25227339e-03,  3.42365366e-01, -1.37708134e-01, -2.05250528e+00,
  1.64503783e-03,  2.33119797e+00,  7.79912524e-01])
    
    num_steps = int(10/0.02 +1) 
    robot = Robot()
    #robot.plot_end_effector_trajectory(traj)
    traj_0_1 = robot.compute_joint_trajectory( q0, q1, num_steps)
    traj_1_2 = robot.compute_joint_trajectory( q1, q2, num_steps)
    traj_2_3 = robot.compute_joint_trajectory( q2, q3, num_steps)
    traj_3_4 = robot.compute_joint_trajectory( q3, q0, num_steps)
    franka = Franka16384()
    franka.set_config(q0)
    franka.follow_trajectory(traj_0_1)
    franka.follow_trajectory(traj_1_2)
    franka.follow_trajectory(traj_2_3)
    franka.follow_trajectory(traj_3_4)



if __name__ == '__main__':
    main()
    
    
    
    
    
    """
    First traj works 
    [INFO] [1763507204.468936]: Velocity between steps 499 and 500: [8.73907902e-05 1.07324995e-04 6.02496509e-05 9.40319947e-05
 6.69491626e-05 2.16759600e-04 1.45164757e-04]
[Franka16384] Trajectory execution complete.
Traceback (most recent call last):
  File "milestone-2.py", line 387, in <module>
    main()
  File "milestone-2.py", line 380, in main
    franka.follow_trajectory(traj_1_2)
  File "/home/student/Documents/frankapy/Miletstone2/FrankaRobot16384.py", line 85, in follow_trajectory
    assert np.allclose(joints_traj[0], current_joint_state, atol=0.05), \
AssertionError: Initial joint configuration does not match current state. Difference: [0.08562336 0.10822206 0.05949807 0.0932971  0.06886288 0.22225433
 0.16116457]



    joints 4
[ 1.25227339e-03  3.42365366e-01 -1.37708134e-01 -2.05250528e+00
  1.64503783e-03  2.33119797e+00  7.79912524e-01]
  
  joints 3
[ 0.17105211  0.01692119 -0.53377664 -1.89941006  0.07431345  1.66426027
  0.32460998]
  
  joints 2: 
[ 0.17939389 -0.09461021  0.13338035 -1.82398707  0.12518016  1.6630852
  1.07733918]
  
  joints 1: 
[ 1.55361583e-02  1.06624155e-01  2.04122546e-02 -2.00029706e+00
 -3.49519799e-04  2.06950945e+00  8.05155261e-01]
    """