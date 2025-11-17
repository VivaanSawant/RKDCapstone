import numpy as np
import matplotlib.pyplot as plt

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
            
            interp = 0.0;
            
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
        # return traj
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

        raise NotImplementedError("Implement compute_jacobian_analytical")

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
        T0 = ... # Compute FK using self.forward_kinematics
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

        raise NotImplementedError("Implement compute_jacobian_numerical")