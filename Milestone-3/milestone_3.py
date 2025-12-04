import numpy as np
import matplotlib.pyplot as plt
from FrankaRobot16384 import Franka16384

def wait_for_user(prompt="[Press 'n' for next, 'r' to retry, or 'q' to quit]: "):
    """Pause execution and wait for valid user input."""
    while True:
        user_input = input(prompt).strip().lower()
        if user_input in ['n', 'r', 'q']:
            return user_input
        print("Invalid input. Please press 'n', 'r', or 'q'.")


# Utility functions for quaternion and rotation matrix conversions
def rotation_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion [x,y,z,w]. DO NOT MODIFY."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    else:
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
    return np.array([x, y, z, w])


def quaternion_to_rotation(q):
    """Convert quaternion [x,y,z,w] to rotation matrix. DO NOT MODIFY."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)]
    ])



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


        self.JOINT_LIMITS_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.JOINT_LIMITS_MAX =  np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

        self.dh_parameters = np.array([
            [0.0,       0.0,        0.333,   0.0],
            [0.0,      -np.pi/2,    0.0,     0.0],
            [0.0,       np.pi/2,    0.316,   0.0],
            [0.0825,   np.pi/2,    0.0,     0.0],
            [-0.0825,   -np.pi/2,    0.384,   0.0],
            [0.0,       np.pi/2,    0.0,     0.0],
            [0.088,     np.pi/2,    0.2104, -np.pi/4]
        ])

    
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
        print(displacement)

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


    def plot_joint_pos_and_vel(self,traj, T=4.0):
        traj = np.asarray(traj)
        N, dof = traj.shape

        # Time vector
        t = np.linspace(0, T, N)
        dt = t[1] - t[0]

        # Joint velocities (shape (N-1, 7))
        vel = np.diff(traj, axis=0) / dt
        t_vel = t[:-1]

        # -------------------------------------------------------
        # FIGURE 1: Joint Positions
        # -------------------------------------------------------
        plt.figure(figsize=(12, 5))
        for j in range(dof):
            plt.plot(t, traj[:, j], label=f'Joint {j+1}')
        plt.title("Joint Position Trajectory")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (rad)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        # -------------------------------------------------------
        # FIGURE 2: Joint Velocities
        # -------------------------------------------------------
        plt.figure(figsize=(12, 5))
        for j in range(dof):
            plt.plot(t_vel, vel[:, j], label=f'Joint {j+1}')
        plt.title("Joint Velocity Trajectory")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (rad/s)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

    # ---------------------------------------------------------------
    # TODO: Compute the Jacobian analytically
    # ---------------------------------------------------------------
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

            Rerror =  R0.T @ R1 # rototaion
            skew = (Rerror - Rerror.T) / (2 * delta)
            dtheta = np.array([
                skew[2, 1],
                skew[0, 2],
                skew[1, 0]
            ])
            J[3:6, i] = dtheta
        return J


###### TODO MILESTONE 3 EXTENSIONS BELOW ######
    # ------------------------------------------------------------
    # Numerical IK (Jacobian-based)
    # ------------------------------------------------------------
    def inverse_kinematics_numerical(self,
                                     q_init: np.ndarray,       # shape (7,)
                                     T_target: np.ndarray,     # shape (4,4)
                                     step_size: float = 0.1,
                                     max_iters: int = 5000,
                                     eps_pos: float = 1e-4,
                                     eps_rot: float = 1e-4):
        """
        Numerical IK using iterative updates and the Jacobian.

        Parameters
        ----------
        q_init : np.ndarray, shape (7,)
            Initial joint guess.
        T_target : np.ndarray, shape (4,4)
            Desired end-effector pose.
        step_size : float
        max_iters : int (keep these reasonably high to ensure convergence)
        eps_pos : float
            Position convergence threshold (meters).
        eps_rot : float
            Orientation convergence threshold (radians).

        Returns
        -------
        q : np.ndarray, shape (7,)
            Final joint configuration that approximates the target pose.

        Hints
        -----
        - Use forward_kinematics() to compute current T.
        - Position error = T_target[:3,3] − T_current[:3,3]
        - Orientation error:
            R_err = R_current.T @ R_target
            Convert to rotation vector using:
                axis = [R_err[2,1]-R_err[1,2], ...] etc.
                angle = arccos((trace(R_err)-1)/2)
                rot_vec = axis * angle
        - Build error vector e = [pos_error ; rot_vec]
        - Use compute_jacobian_numerical(q)
        - Joint update:  q = q + step_size * (J.T @ e)
        - clamp q within joint limits after each update , see self.JOINT_LIMITS_MIN/MAX an use np.clip(q, min, max)
        - Check convergence: if ||pos_error|| < eps_pos and ||rot_vec|| < eps_rot: break
        """
        q = q_init.copy() # make a new copy
        
        targetTranslation = T_target[:3, 3]
        targetRotation = T_target[:3, :3] # this has colon
        
        for _ in range(max_iters): # iteratively go down
            T_curr = self.forward_kinematics(self.dh_parameters, q)
            
            currentTranslation = T_curr[:3, 3]
            currentRotation = T_curr[:3, :3] # this has colon
            
            # Position error = T_target[:3,3] − T_current[:3,3]
            position_error = targetTranslation - currentTranslation
            
            '''
            Orientation error:
            R_err = R_current.T @ R_target
            Convert to rotation vector using:
                axis = [R_err[2,1]-R_err[1,2], ...] etc.
                angle = arccos((trace(R_err)-1)/2)
                rot_vec = axis * angle
            '''
            
            orientation_error = currentRotation.T @ targetRotation
            diagonalSum = np.trace(orientation_error)
            angle = np.arccos(np.clip((diagonalSum - 1) / 2, -1.0, 1.0)) 
            
            if angle < 1e-9: # avoids divides by 0
                rot_vec = np.zeros(3)
            else:
                axis = np.array([
                    orientation_error[2, 1] - orientation_error[1, 2],
                    orientation_error[0, 2] - orientation_error[2, 0],
                    orientation_error[1, 0] - orientation_error[0, 1]
                ]) / (2 * np.sin(angle))

                # rot_vec = axis * angle
                rot_vec = axis * angle
            
            # Check convergence: if ||pos_error|| < eps_pos and ||rot_vec|| < eps_rot: break
            
            if np.linalg.norm(position_error) < eps_pos and np.linalg.norm(rot_vec) < eps_rot:
                break
            
            e = np.hstack((position_error, 0.01*rot_vec))
            J = self.compute_jacobian_numerical(q)
            #q = q + step_size * (J.T @ e)
            lam = 1e-4   # damping term
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lam * np.eye(6))
            q = q + J_pinv @ e
            #q = np.clip(q, self.JOINT_LIMITS_MIN, self.JOINT_LIMITS_MAX)
        return q

            
            
            
            
        
        
        # raise NotImplementedError("TODO: Implement numerical IK")







    # ------------------------------------------------------------
    # DRAW LINE: end-effector interpolation + IK (TODO)
    # ------------------------------------------------------------
    def draw_line(
            self,
            q_start: np.ndarray,   # (7,)
            dp: np.ndarray,        # (3,) displacement in EE space
            num_steps: int = 30):
        """
        Follow a straight line in **end-effector space**, using a displacement
        from the current EE position instead of an absolute goal.

        Returns
        -------
        traj : np.ndarray, shape (num_steps, 7)

        Hints
        -----
        - Compute T_start = FK(q_start)
        - Extract p_start and R_start (fix orientation!)
        - Compute p_goal = p_start + dp   <-- NEW
        - Interpolate translation only:
            p(s) = (1 - s)*p_start + s*p_goal
        - Build T_s = [R_start, p(s)]
        - Call IK at each step (warm start with previous solution)
        """

        T_start = self.forward_kinematics(self.dh_parameters, q_start)
        
        # Extract p_start and R_start (fix orientation!)
        
        p_start = T_start[:3, 3]
        R_start = T_start[:3, :3]
        
        # Compute p_goal = p_start + dp   <-- NEW
        p_goal = p_start + dp
        
        # build trajecotry matrix
        
        traj = np.zeros((num_steps, 7))
        traj[0] = q_start.copy()
        
        q_prev = q_start.copy() # first IK step
        
        for i in range(1, num_steps):
            s = i / (num_steps - 1)
        
            '''
            Interpolate translation only:
                p(s) = (1 - s)*p_start + s*p_goal
            '''
            ps = (1 - s)*p_start + s*p_goal
            
            # Build T_s = [R_start, p(s)]
            
            T_s = np.eye(4)
            T_s[:3, :3] = R_start
            T_s[:3, 3] = ps
            
            q_next = self.inverse_kinematics_numerical(q_prev, T_s, step_size = 0.1,
                                        max_iters = 2000) # this was taken from the previuos part
            
            traj[i] = q_next
            q_prev = q_next
            
        return traj
            
        
        
        
        # raise NotImplementedError("TODO: Implement draw_line")



    # ------------------------------------------------------------
    # DRAW CURVE: arbitrary end-effector interpolation + IK (TODO)
    # ------------------------------------------------------------
    def draw_curve(self,
                q_start: np.ndarray,       # (7,) starting joint configuration
                curve_fn,                  # lambda s -> 4x4 pose (EE transform)
                num_steps: int = 40):
        """
        Follow a parametric curve in end-effector space using IK.

        Returns
        -------
        traj : np.ndarray, shape (num_steps, 7)

        Hints
        ---------------------------
        1. curve_fn is a function/lambda taking a single float s ∈ [0,1]
        and returning a **4x4 homogeneous transform** representing the
        desired EE pose at that point.

        Example:
            curve_fn(0.0)  -> start pose (4x4)
            curve_fn(0.5)  -> midpoint pose (4x4)
            curve_fn(1.0)  -> end pose (4x4)

        2. To follow the curve, do a loop over s values:
            s_vals = np.linspace(0, 1, num_steps)

        For each s:
            - Compute T_s = curve_fn(s)
            - Extract R_s = T_s[:3, :3] and p_s = T_s[:3, 3]
            - Solve IK: q_next = inverse_kinematics_numerical(q_prev, T_s)
                (warm-start IK with q_prev from previous step)
            - Append q_next to trajectory
            - Update q_prev = q_next

        3. Example curve (semicircle in XY plane with fixed Z and fixed orientation):
            ```python
            import numpy as np

            R_fixed = np.eye(3)
            p_center = np.array([0.5, 0.0, 0.3])
            radius = 0.2

            curve_fn = lambda s: np.block([
                [R_fixed, np.array([[p_center[0] + radius*np.cos(np.pi*s)],
                                    [p_center[1] + radius*np.sin(np.pi*s)],
                                    [p_center[2]]])],
                [0, 0, 0, 1]
            ])
            ```
            - s=0 → start of semicircle
            - s=1 → end of semicircle

            You can generate any curve: spirals, hearts, circles, lines, 3D shapes…

        4. Important constraints:
            - curve_fn(s) **must return a valid 4×4 transform** every time.
            - If orientation is fixed: R_s is constant.
            - If orientation changes, T_s must include that rotation.
            - The robot solves IK for every step to obtain the joint-space path.

        Summary:
            - s ∈ [0,1] is normalized curve progress.
            - curve_fn(s) defines the desired EE pose at that point.
            - IK converts pose → joint configuration.
            - Collect all q values to form the trajectory.
        """
        traj = np.zeros((num_steps, 7))
        traj[0] = q_start.copy()
        
        q_prev = q_start.copy() 
        s_vals = np.linspace(0, 1, num_steps)
        for i, s in enumerate(s_vals):
            if(i==0):
                continue
            T_s = curve_fn(s)
            q_next = self.inverse_kinematics_numerical(q_prev, T_s, step_size=0.001, max_iters=2000)
            traj[i] = q_next
            q_prev = q_next
        return traj
        #raise NotImplementedError("TODO: Implement draw_curve using curve_fn")





###################### BONUS ##########################




    # ------------------------------------------------------------
    # SLERP: Quaternion Interpolation
    # ------------------------------------------------------------
    def slerp(self,
              q1: np.ndarray,   # shape (4,)
              q2: np.ndarray,   # shape (4,)
              s: float):
        """
        Spherical linear interpolation (SLERP) between two quaternions.

        Parameters
        ----------
        q1, q2 : np.ndarray, shape (4,)
            Input quaternions in (x,y,z,w) format.
        s : float in [0,1]
            Interpolation parameter.

        Returns
        -------
        q_out : np.ndarray, shape (4,)
            Interpolated quaternion.

        Hints
        -----
        - Normalize inputs q1, q2.
        - Dot = q1 · q2
        - If dot < 0:
            q2 = -q2 (take shortest path)
        - If angle small → use LERP fallback
        - Implement:
            theta = arccos(dot)
            sin_theta = sin(theta)
            q = (sin((1-s)*theta)/sin_theta)*q1 + (sin(s*theta)/sin_theta)*q2
        """
        #define small_angle
        small_angle = 1e-6

        #normalize q1, q2
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        # compute ”dot product” of q0 and q1 to get cos(halfAngle)
        #cosHalfAngle = q1.a*q2.a + q1.b*q2.b + q1.c*q2.c + q1.d*q2.d
        cosHalfAngle = np.dot(q1,q2)
        # interpolating around negative cosine goes the long way around
        if cosHalfAngle < 0:
            q2, cosHalfAngle = -q2, -cosHalfAngle

        if cosHalfAngle > (1-small_angle):
            # USE LERP Fallback
            q = (1 - s) * q1 + s * q2
            return q / np.linalg.norm(q)
        # cos(0) = 1, the quaternions are the same rotation
        if cosHalfAngle >= 1:
            return q1
        # get the half angle, and use Pythagorean to get sin
        halfAngle = acos(cosHalfAngle)
        sinHalfAngle = sqrt(1.0 - cosHalfAngle*cosHalfAngle)

        
        # interpolate
        p1 = sin((1-s) * halfAngle) / sinHalfAngle
        p2 = sin(s * halfAngle) / sinHalfAngle;
        return q1*p1 + q2*p2




    # ------------------------------------------------------------
    # Torch Rotation Trajectory using SLERP + IK (TODO)
    # ------------------------------------------------------------
    def rotate_torch_trajectory(self,
                                q_start: np.ndarray,     # (7,)
                                R_goal: np.ndarray,      # (3,3)
                                num_steps: int = 20,
                                direction: str = "up"):
        """
        Compute a joint trajectory that rotates the torch/light
        while keeping its position fixed.

        Returns
        -------
        traj : np.ndarray, shape (num_steps, 7)

        Hints
        -----
        - Get T_start = FK(q_start)
        - Extract p_start and R_start
        - For s in linspace(0,1,num_steps):

            Rotation interpolation from R_start → R_goal.
            - Convert R_start, R_goal to quaternions
            - q_s = slerp(q_start, q_goal, s)
            - Convert back to 3×3 rotation matrix
            - Build T_s = [R_s, p_start]
            - q_next = inverse_kinematics_numerical(q_prev, T_s)
            - Append each q_next to trajectory
        """
        traj = []
        T_start = self.forward_kinematics(self.dh_parameters, q_start)
        R_start = T_start[:3, :3]   # top-left 3×3 block
        p_start = T_start[:3, 3] 
        s_vals = np.linspace(0, 1, num_steps)
        quat_start = rotation_to_quaternion(R_start)
        quat_goal  = rotation_to_quaternion(R_goal)
        q_prev = q_start.copy()
        for i, s in enumerate(s_vals):
            
            q_s = slerp(quat_start, quat_goal, s)
            R_s = quaternion_to_rotation(q_s)
            T_s = np.eye(4)
            T_s[:3, :3] = R_s
            T_s[:3, 3]  = p_start


            q_next = self.inverse_kinematics_numerical(q_prev, T_s)
            
            traj.append(q_next)
            q_prev = q_next
        return np.array(traj)
    def running_action(self, q):
        #robot = Robot()
        franka = Franka16384()
        franka.follow_trajectory(q)
        while True:
            user_choice = wait_for_user("Press 'n' to continue to place or 'r' to retry move: ")
            if user_choice == 'n':
                break
            elif user_choice == 'r':
                print("[Retrying move to place pose...]")
                franka.follow_trajectory(q)
            elif user_choice == 'q':
                print("[Exiting program]")
                return
        return 
    
    def draw_full_vertical_circle(self, q_start, radius=0.03, steps_per_arc=200):
        """
        Draw a full 360° vertical circle (Y–Z plane) by stitching 4 smaller arcs.
        
        Parameters
        ----------
        q_start : np.ndarray, shape (7,)
            Starting joint configuration.
        radius : float
            Radius of the vertical circle (meters).
        steps_per_arc : int
            Number of IK steps per 90° arc.

        Returns
        -------
        traj_full : np.ndarray, shape (4*steps_per_arc, 7)
            Joint trajectory covering the full circle.
        """

        # ---- Helper to generate a Y-Z vertical arc ----
        def make_arc_fn(R_fixed, p_start, radius, theta0, theta1):
            return lambda s: np.block([
                [R_fixed,
                np.array([
                    [p_start[0]],                                      # x stays constant
                    [p_start[1] + radius*np.cos(theta0 + (theta1-theta0)*s)],
                    [p_start[2] + radius*np.sin(theta0 + (theta1-theta0)*s)]
                ])],
                [0,0,0,1]
            ])

        # ---- Initial FK pose ----
        T_start = self.forward_kinematics(self.dh_parameters, q_start)
        p_start = T_start[:3, 3]
        R_fixed = T_start[:3, :3]

        # ---- Build the 4 arcs ----
        arcs = []
        q_prev = q_start

        # Angles for 4×90° arcs
        theta_pairs = [
            (0*np.pi/2, 1*np.pi/2),
            (1*np.pi/2, 2*np.pi/2),
            (2*np.pi/2, 3*np.pi/2),
            (3*np.pi/2, 4*np.pi/2)
        ]

        for theta0, theta1 in theta_pairs:
            curve_fn = make_arc_fn(R_fixed, p_start, radius, theta0, theta1)
            traj_arc = self.draw_curve(q_prev, curve_fn, num_steps=steps_per_arc)

            # append this arc
            arcs.append(traj_arc)
            q_prev = traj_arc[-1]        # warm-start next arc

        # ---- Combine all arcs ----
        traj_full = np.vstack(arcs)

        return traj_full




        ### To run it: 
    '''
    traj_circle = robot.draw_full_vertical_circle(q_start, radius=0.03, steps_per_arc=200)
    robot.follow_trajectory(traj_circle)
    '''

    
    def picking_flashlight(self, q_home, q_above_pick_place, q_pick_place, q_facing_camera, num_steps):
        robot = Robot()
        franka = Franka16384()

        traj_home_above = robot.compute_joint_trajectory( q_home, q_above_pick_place, num_steps)            
        #open grippers
        traj_above_pick = robot.compute_joint_trajectory( q_above_pick_place, q_pick_place, num_steps)
        #close grippers
        traj_pick_above = robot.compute_joint_trajectory( q_pick_place, q_above_pick_place , num_steps)
        #traj_above_place = robot.compute_joint_trajectory( q_above_pick_place , q_pick_place, num_steps)
        
        traj_above_camera = robot.compute_joint_trajectory(q_home, q_facing_camera, num_steps)
        '''
        print("\n[Step 2] Moving to a position above the flashlight...")
        robot.running_action(traj_home_above)
        
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
        robot.running_action(traj_above_pick)

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
        robot.running_action(traj_pick_above)
        '''
        #q_facing_camera   
        print("\n[Step 7] Moving the flashlight to face the camera...")
        robot.running_action(traj_above_camera)

    
    def flashlight(self, q_home, q_above_pick_place, q_pick_place, q_facing_camera, q_lineTop, q_lineMiddle, q_lineBottom, q_bin, num_steps):
        robot = Robot()
        franka = Franka16384()
        traj_home_above = robot.compute_joint_trajectory( q_home, q_above_pick_place, num_steps)
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
        #step2
        robot.picking_flashlight(q_home, q_above_pick_place, q_pick_place, q_facing_camera, num_steps)
        
        print("\n[Step 3 Moving the flashlight in the largest circle...")
        R_fixed = robot.forward_kinematics(self.dh_parameters, q_facing_camera)[:3,:3]
        p_center = robot.forward_kinematics(self.dh_parameters, q_facing_camera)[:3,3]
 
        print(p_center)
        radius = 0.032

        curve_fn = lambda s: np.block([
            [R_fixed, (np.array([[p_center[0]],
                                 [p_center[1] + radius*np.cos(np.pi*s)-radius],
                                [p_center[2] + radius*np.sin(np.pi*s)]])).reshape(3,1)],
            [0, 0, 0, 1]
        ])
        
        traj_curve = robot.draw_curve(q_facing_camera, curve_fn, 100)

        #######
        robot.running_action(traj_curve)


        R_fixed2 = robot.forward_kinematics(self.dh_parameters, (traj_curve[-1]))[:3,:3]
        p_center2 = robot.forward_kinematics(self.dh_parameters, (traj_curve[-1]))[:3,3]
 
        radius2 = 0.032

        curve_fn2 = lambda s: np.block([
            [R_fixed2, (np.array([[p_center2[0]],
                                 [p_center2[1] + radius2*np.cos(np.pi*-1*s)-radius2],
                                [p_center2[2] + radius2*np.sin(np.pi*-1*s)]])).reshape(3,1)],
            [0, 0, 0, 1]
        ])
        
        traj_curve2 = robot.draw_curve((traj_curve[-1]), curve_fn2, 100)

        #######
        robot.running_action(traj_curve2)

        R_fixed3 = robot.forward_kinematics(self.dh_parameters, (traj_curve2[-1]))[:3,:3]
        p_center3 = robot.forward_kinematics(self.dh_parameters, (traj_curve2[-1]))[:3,3]
 
        radius3 = 0.032

        curve_fn3 = lambda s: np.block([
            [R_fixed3, (np.array([[p_center3[0]],
                                 [p_center3[1] + radius3*np.cos(np.pi*s)-radius3],
                                [p_center3[2] + radius3*np.sin(np.pi*s)]])).reshape(3,1)],
            [0, 0, 0, 1]
        ])
        
        traj_curve3 = robot.draw_curve((traj_curve2[-1]), curve_fn3, 100)

        #######
        robot.running_action(traj_curve3)

        #q_lineTop
        curve_to_line1_traj = robot.compute_joint_trajectory((traj_curve2[-1]), q_lineTop, num_steps)
        #dp_line1 = 
        traj_line12 = robot.draw_line(q_lineTop, dp, num_steps)
        line1_to_line2_traj = robot.compute_joint_trajectory(q_lineTop, q_lineMiddle, num_steps)
        line2_to_line3_traj = robot.compute_joint_trajectory(q_lineMiddle, q_lineBottom, num_steps)
        q_bin_traj = robot.compute_joint_trajectory(q_lineBottom, q_bin, num_steps)
  
        robot.running_action(q_bin_traj)

        '''
        print("\n[Step 9] Moving the flashlight to start of the smallest circle...")


        print("\n[Step 10] Moving the flashlight in the smallest circle...")

        print("\n[Step 13] Moving the flashlight to the bin...")

        print("\n[Step 11] Dropping flashlight in bin...")
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

        print("\n[Step 12] Picking up new flashlight...")
        robot.picking_flashlight(traj_home_above, traj_above_pick, traj_pick_above, traj_above_camera)

        print("\n[Step 13] Moving the flashlight to the medium circle...")

        print("\n[Step 13] Moving the flashlight in the medium circle...")

        print("\n[Step 13] Moving the flashlight to the bin...")

        print("\n[Step 14] Dropping flashlight in bin...")
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

        print("\n[Step 15] Picking up new flashlight...")
        robot.picking_flashlight(traj_home_above, traj_above_pick, traj_pick_above, traj_above_camera)

        print("\n[Step 16] Iteratively drawing vertical lines at a rotation at increments of pi/4...")
        for i in range(8):
            angle = i * (np.pi / 4)
            R_goal = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle),  np.cos(angle), 0],
                [0,              0,             1]
            ])
            traj_rotate = robot.rotate_torch_trajectory(q_facing_camera, R_goal, num_steps, direction="up")
            robot.running_action(traj_rotate)
            print(f"\n[Step 16.{i+1}] Completed rotation to angle {angle} radians.")

        print("\n[Step 13] Moving the flashlight to the bin...")

        print("\n[Step 17] Dropping flashlight in bin...")
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
    '''

                
    '''
    # 2. Choose a displacement dp (move 10 cm along z)
        dp = np.array([0.0, 0.0, -0.01])     

        # 3. Call draw_line
        traj_line = robot.draw_line(q_facing_camera, dp, num_steps)

        print(traj_line[-1])
        R_fixed = robot.forward_kinematics(self.dh_parameters, traj_line[-1])[:3,:3]
        p_center = robot.forward_kinematics(self.dh_parameters, traj_line[-1])[:3,3]
 
        print(p_center)
        radius = 0.01

        curve_fn = lambda s: np.block([
            [R_fixed, (np.array([[p_center[0] + radius*np.cos(np.pi*s)-radius],
                                [p_center[1] + radius*np.sin(np.pi*s)],
                                [p_center[2]]])).reshape(3,1)],
            [0, 0, 0, 1]flashlight_far
        ])
        print(curve_fn(0))
        traj_curve = robot.draw_curve(traj_line[-1 ], curve_fn, 100)
        

        robot.picking_flashlight(traj_home_above, traj_above_pick, traj_pick_above, traj_above_camera)
    '''

def main():
    robot = Robot()   # assuming your class is named Robot()

    #flashlight test
    num_steps = int(robot.total_time/0.02)
    
    q_home = np.array([-0.00608366, -0.95394664, -0.03228957, -2.59348415, -0.00837678,  1.59325033,
  0.76390718])
    q_above_flashlight = np.array([-0.18845576,  0.32761044, -0.16718853, -2.13337842,  0.13734175,  2.48372093,
  0.32061945])
    q_pick_place = np.array([ 0.00482188,  0.42789345, -0.00878065, -2.25411856, -0.00964446,  2.63687885,
  0.78377835])
    
    # 1. Choose a starting configuration
    q_curvesStart = np.array([ 0.15653988, -0.39924142,  0.04378931, -2.79378656,  0.1100186,   3.45262439, 0.85302329])
    q_lineTop = np.array([ 0.16016253, -0.21508658,  0.03883015, -2.79192153,  0.11008639,  3.45258017, 0.85303423])
    q_lineMiddle = np.array([ 0.17390262, -0.0620786,   0.04324487, -2.7910104,   0.11009168,  3.45224358, 0.85364052])
    q_lineBottom = np.array([ 0.17099069,  0.05200516,  0.03982096, -2.79224346,  0.11008882,  3.45220808, 0.85302305])
    #[-0.29728245, -0.49473546,  0.08392697, -2.81276122, -0.34758527,  3.74226282, 1.07860857]

    flashlight_far = np.array([-0.15362715,  0.76493927, -0.39913538, -1.79927101, -0.19962317,  1.91157644,
  0.27320951])

    flashlight_middle = np.array(
    [-0.09710638,  0.64428856, -0.33450619, -1.94583503, -0.06465234,  2.10779242,
  0.43122558])

    flashlight_near = np.array([-0.23963659,  1.00373385, -0.26781676, -1.28161212, -0.19933129,  1.59947069,
  0.31113862])

    q_bin = np.array([ 0.16470852,  0.35613679,  0.0911824,  -2.26084658,  0.13332707,  2.66035794,
  0.84524366])


    robot.flashlight(q_home, q_drawStart, flashlight_middle, q_curvesStart, q_lineTop, q_lineMiddle, q_lineBottom,  q_bin, num_steps)


    

if __name__ == '__main__':
    main()


'''

lowest to highest  
  line 1: [ 0.17099069  0.05200516  0.03982096 -2.79224346  0.11008882  3.45220808
  0.85302305]

  start curve: [ 0.17390262 -0.0620786   0.04324487 -2.7910104   0.11009168  3.45224358
  0.85364052]

  line 2: [ 0.16016253 -0.21508658  0.03883015 -2.79192153  0.11008639  3.45258017
  0.85303423]

    line 3: [ 0.15653988 -0.39924142  0.04378931 -2.79378656  0.1100186   3.45262439
  0.85302329]


'''
"""
1. make a circle of size 10cm in the XY plane at Z=0 with one color
2. make a circle of size 7cm in the YZ plane at X=0 with another color
3. make a circle of size 5cm in the XZ plane at Y=0 with a third color
4. go to the center and move in a straight line to the each of the three circles
   at increments of pi/4 
"""
