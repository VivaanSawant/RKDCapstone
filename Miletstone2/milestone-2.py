import numpy as np
import matplotlib.pyplot as plt
#from Miletstone2.FrankaRobot16384 import Franka16384

def wait_for_user(prompt="[Press 'n' for next, 'r' to retry, or 'q' to quit]: "):
    """Pause execution and wait for valid user input."""
    while True:
        user_input = input(prompt).strip().lower()
        if user_input in ['n', 'r', 'q']:
            return user_input
        print("Invalid input. Please press 'n', 'r', or 'q'.")




class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7

        # Example DH parameters for Franka Emika 7-DOF robot
        # TODO: Fill in with actual DH parameters copy from milestone 1
        self.dh_parameters = np.array([
            [0.0,       0.0,        0.333,   0.0],
            [0.0,      -np.pi/2,    0.0,     0.0],
            [0.0,       np.pi/2,    0.316,   0.0],
            [0.0825,   np.pi/2,    0.0,     0.0],
            [-0.0825,   -np.pi/2,    0.384,   0.0],
            [0.0,       np.pi/2,    0.0,     0.0],
            [0.088,     np.pi/2,    0.2104, -np.pi/4]
        ])

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
        q0, q1 = np.array(q0), np.array(q1)
        displacement = q1 - q0

        T = self.total_time
        ta = self.accel_time # 0.5
        td = self.decel_time # 0.5
        tc = T - ta - td     # 1.0 (according to writeup)

        # Use endpoint = true
        time_samples = np.linspace(0, T, num_steps, endpoint=True)

        vmax = 1.0 / (tc + 0.5*ta + 0.5*td)
        a = vmax / ta
        d = vmax / td

        traj = np.zeros((num_steps, 7))

        for i, t in enumerate(time_samples):

            if t <= ta:
                s = 0.5 * a * t**2
            elif t <= ta + tc:
                s = 0.5 * vmax * ta + vmax * (t - ta)
            else:
                tdec = t - (ta + tc)
                sbefore = 0.5 * vmax * ta + vmax * tc
                s = sbefore + vmax * tdec - 0.5 * d * tdec**2
            traj[i, :] = q0 + s * displacement

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
        Ts = []            
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
        T0 = self.forward_kinematics(self.dh_parameters, thetas)
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

            T1 = self.forward_kinematics(self.dh_parameters, thetasPerturbed)
            pos1 = T1[:3, 3]
            R1 = T1[:3, :3]
            Pdifference = (pos1 - pos0) / delta # postion
            J[0:3, i] = Pdifference

            Rerror =  R1 @ R0.T # rototaion
            skew = (Rerror - Rerror.T) / (2 * delta)
            dtheta = np.array([
                skew[2, 1],
                skew[0, 2],
                skew[1, 0]
            ])
            J[3:6, i] = dtheta
        return J
        #raise NotImplementedError("Implement compute_jacobian_numerical")
    def flashlight(self, q_home, q_above_pick_place, q_pick_place, num_steps):
        robot = Robot()
        franka = Franka16384()
        traj_home_above = robot.compute_joint_trajectory( q_home, q_above_pick_place, num_steps)            
        #open grippers
        traj_above_pick = robot.compute_joint_trajectory( q_above_pick_place, q_pick_place, num_steps)
        #close grippers
        traj_pick_above = robot.compute_joint_trajectory( q_pick_place, q_above_pick_place , num_steps)
        traj_above_place = robot.compute_joint_trajectory( q_above_pick_place , q_pick_place, num_steps)
        #open grippers
        print("\n[Step 1] Moving to home position...")
        franka.set_config(q_home)
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.set_config(q_home)
            elif user_choice == 'q':
                print("[Exiting program]")
                return

            
        print("\n[Step 2] Moving to a position above the flashlight...")
        franka.follow_trajectory(traj_home_above)
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.follow_trajectory(traj_home_above)
            elif user_choice == 'q':
                print("[Exiting program]")
                return
        
        print("\n[Step 3] Opening the grippers ...")
        franka.open_gripper()
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.open_gripper()
            elif user_choice == 'q':
                print("[Exiting program]")
                return

        
        print("\n[Step 4] Moving to pick up the flashlight...")
        franka.follow_trajectory(traj_above_pick)
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.follow_trajectory(traj_above_pick)
            elif user_choice == 'q':
                print("[Exiting program]")
                return

        print("\n[Step 5] Closing the grippers...")
        franka.close_gripper()
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.close_gripper()
            elif user_choice == 'q':
                print("[Exiting program]")
                return

        print("\n[Step 6] Moving the flashlight up...")
        franka.follow_trajectory(traj_pick_above)
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.follow_trajectory(traj_pick_above)
            elif user_choice == 'q':
                print("[Exiting program]")
                return

        print("\n[Step 7] Moving the flashlight back down...")
        franka.follow_trajectory(traj_above_place)
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.follow_trajectory(traj_above_place)
            elif user_choice == 'q':
                print("[Exiting program]")
                return

        print("\n[Step 8] Opening the grippers ...")
        franka.open_gripper()
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.open_gripper()
            elif user_choice == 'q':
                print("[Exiting program]")
                return

        return

def main():
    robot = Robot()

    '''
    Plotting the trajectory:
    traj = np.array([[0.02204015, -0.35898457,  0.02613999, -2.364214, 0.00274931,  2.04544125, 0.82331108],
    [-1.41190431, -1.12662768,  1.05950991, -2.3611671,0.92804899,  1.57607987,  0.0403548 ],
    [-0.07447891, -0.03003863, -0.18793599, -2.47069798, 0.09206394, 2.57397382, 0.39425526]
    ])
    robot.plot_end_effector_trajectory(traj)'''

    '''
    #Moving the robot in a loop (4 positions)
    q0 = np.array([-0.0819264,  -0.13599538, 0.0575315,  -2.36497335,  0.0082773,   2.2991974, 0.67502163])
    q1 = np.array([-0.05729219, -0.42088033,  0.09961394, -2.63788948,  0.00826903,  2.2058541, 0.73903088])
    q2 = np.array([-0.20558467, -0.20446108, -0.02575163, -2.63800154,  0.00706992,  2.55150997, 0.69112535])
    q3 = np.array([-0.11125982, -0.3682046,   0.0764298,  -2.58438404,  0.00711509,  2.82580872, 0.70104034])
    
    num_steps = int(robot.total_time/0.02)
    
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
    '''
    '''
    #flashlight test
    num_steps = int(robot.total_time/0.02)
    q_home = np.array([-0.00608366, -0.95394664, -0.03228957, -2.59348415, -0.00837678,  1.59325033,
  0.76390718])
    q_above_pick_place = np.array([-2.60259668e-03,  2.41709376e-01, -1.85602809e-03, -2.22746265e+00,
 -7.81279907e-03,  2.42024084e+00,  7.84722198e-01])
    q_pick_place = np.array([ 0.00482188,  0.42789345, -0.00878065, -2.25411856, -0.00964446,  2.63687885,
  0.78377835])
    robot.flashlight(q_home, q_above_pick_place, q_pick_place, num_steps) 
    '''
    thetas = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
    #thetas2 = np.array([-0.05729219, -0.42088033,  0.09961394, -2.63788948,  0.00826903,  2.2058541, 0.73903088])
    numerical_jacobian = robot.compute_jacobian_numerical(thetas, delta=1e-4)
    analytical_jacobian = robot.compute_jacobian_analytical(thetas)
    print(f"numerical jacobian: \n\n{numerical_jacobian}")
    print(f"analytical jacobian:\n\n {analytical_jacobian}")
    difference = numerical_jacobian - analytical_jacobian
    print(f"Difference between numerical and analytical Jacobians:\n\n {difference}")
  
    
    
    




if __name__ == '__main__':
    main()
    
    



    """
    joints: 
[-0.0819264,  -0.13599538, 0.0575315,  -2.36497335,  0.0082773,   2.2991974, 0.67502163]
Enter a number: 2
joints: 
[-0.05729219, -0.42088033,  0.09961394, -2.63788948,  0.00826903,  2.2058541, 0.73903088]
Enter a number: 2
joints: 
[-0.20558467, -0.20446108, -0.02575163, -2.63800154,  0.00706992,  2.55150997, 0.69112535]
Enter a number: 2
joints: 
[-0.11125982, -0.3682046,   0.0764298,  -2.58438404,  0.00711509,  2.82580872, 0.70104034]
    """
    
    
    
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












'''
ANGLES FOR THE FLASHLIGHT TASK:


home: 
[-0.00608366, -0.95394664, -0.03228957, -2.59348415, -0.00837678,  1.59325033,
  0.76390718]

above1 
[-2.60259668e-03,  2.41709376e-01, -1.85602809e-03, -2.22746265e+00,
 -7.81279907e-03,  2.42024084e+00,  7.84722198e-01]

pick
[ 0.00482188,  0.42789345, -0.00878065, -2.25411856, -0.00964446,  2.63687885,
  0.78377835]

above2
[ 0.09792095,  0.315963,    0.17234519, -2.16128569, -0.01907918,  2.47031701,
  1.04222218]

place
[ 0.09401129,  0.51291985,  0.15605813, -2.07454963, -0.01909078,  2.47660728,
  1.04198222]


'''
