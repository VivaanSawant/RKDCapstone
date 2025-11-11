import sys
sys.path.append('../config')
import numpy as np

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
        # --------------- END STUDENT SECTION --------------------------------------------------
    
    

        
    # --------------- END STUDENT SECTION --------------------------------------------------
"""def main():
    #DH parameters for Franka Emika Panda robot
    thetas = np.array([0.02204015, -0.35898457, 0.02613999, -2.364214,
                    0.00274931,  2.04544125, 0.82331108])
    thetas2 = np.array([-1.41190431, -1.12662768,  1.05950991, -2.3611671,
                         0.92804899,  1.57607987,  0.0403548 ])
    
    dh_parameters = np.array([
        [0.0,       0.0,        0.333,   0.0],
        [0.0,      -np.pi/2,    0.0,     0.0],
        [0.0,       np.pi/2,    0.316,   0.0],
        [0.0825,   np.pi/2,    0.0,     0.0],
        [-0.0825,   -np.pi/2,    0.384,   0.0],
        [0.0,       np.pi/2,    0.0,     0.0],
        [0.088,     np.pi/2,    0.2104, -np.pi/4]
    ])
    # dh_parameters = np.array([
    #     [0.0,         0.0,  0.333,  0.0],
    #     [0.0,        -np.pi/2,  0.0,    0.0],
    #     [0.0,         np.pi/2,  0.316,  0.0],
    #     [0.0825,      np.pi/2,  0.0,    0.0],
    #     [-0.0825,    -np.pi/2,  0.384,  0.0],
    #     [0.0,         np.pi/2,  0.0,    0.0],
    #     [0.088,        np.pi/2,  0.0,  0.0],
    #     [0.0,         0.0,      0.107, 0.0]
    # ])  
    robot = Robot()
    value = robot.forward_kinematcis(dh_parameters, thetas)
    print(value)

if __name__ == '__main__':
    main()
        #
    pose: 
Tra: [0.44515306 0.0214124  0.36430572]
 Rot: [[ 0.99914764  0.00748049  0.04035864]
 [ 0.00768266 -0.99994907 -0.00485668]
 [ 0.04032026  0.0051626  -0.99917345]]
 Qtn: [0.00250536 0.99978375 0.00379161 0.02017409]
 from franka_tool to world
Enter a number: 2
joints: 
[ 0.02204015 -0.35898457  0.02613999 -2.364214    0.00274931  2.04544125
  0.82331108]

  
same ee pose as above but different joint configuration
pose: 
Tra: [0.42242995 0.00322342 0.34880284]
 Rot: [[ 0.99972025 -0.00613758  0.02241672]
 [-0.00632291 -0.99993671  0.00820603]
 [ 0.02236494 -0.00834547 -0.99971504]]
 Qtn: [-0.0041382   0.9999215  -0.00311537  0.01119629]
 from franka_tool to world
Enter a number: 2
joints: 
[-1.41190431 -1.12662768  1.05950991 -2.3611671   0.92804899  1.57607987
  0.0403548 ]


(franka) student@iam-mirabel:~/Documents/frankapy$ python examples/milestone-1.py 
THETAS 1
[[ 0.55849829 -0.60956343  0.56259407 -0.01001165]
 [-0.41988863 -0.79266595 -0.44201158  0.60860588]
 [ 0.71538325  0.01063586 -0.69865133  0.40276681]
 [ 0.          0.          0.          1.        ]]
(franka) student@iam-mirabel:~/Documents/frankapy$ python examples/milestone-1.py 

THETAS 2
[[-0.69634206 -0.64610424  0.31250128  0.42529902]
 [ 0.35227737  0.07166689  0.93314764 -0.32068903]
 [-0.62530664  0.75987707  0.17770323  0.50176746]
 [ 0.          0.          0.          1.        ]]
    """


'''
extra tests

pose: 
Tra: [ 0.48296426 -0.12533482  0.2141333 ]
 Rot: [[ 0.98807053  0.05471859  0.14388762]
 [ 0.05263011 -0.99843704  0.01828415]
 [ 0.14466321 -0.0104932  -0.98942511]]
 Qtn: [-0.00721608  0.99698705  0.02691828  0.07235571]
 from franka_tool to world
Enter a number: 2
joints: 
[-0.07447891 -0.03003863 -0.18793599 -2.47069798  0.09206394  2.57397382
  0.39425526]
Enter a number: 1




pose: 
Tra: [ 0.46261766 -0.12069118  0.20511049]
 Rot: [[ 0.98928127 -0.0235963   0.14403794]
 [-0.02591124 -0.99955352  0.01421693]
 [ 0.14363816 -0.01779674 -0.98947004]]
 Qtn: [-0.00802521  0.99728442 -0.01241059  0.07211486]
 from franka_tool to world
Enter a number: 2
joints: 
[ 1.42811806  0.51682877 -1.66892051 -2.4827083   0.87168655  2.37629885
 -0.07177429]
Enter a number: 

'''


"""
NEW POSITION 1
pose: 
Tra: [ 0.53041967 -0.01625034  0.2346408 ]
 Rot: [[ 0.99766966  0.06789834  0.00508119]
 [ 0.06803928 -0.99700119 -0.03660669]
 [ 0.00258042  0.03686711 -0.99931683]]
 Qtn: [0.01838227 0.99924818 0.03400998 0.00191684]
 from franka_tool to world
Enter a number: 2
joints: 
[-0.01480397  0.0641733  -0.0069865  -2.24832218 -0.04913605  2.31775459
  0.72885828]
Enter a number: 1
pose: 
Tra: [ 0.54427966 -0.01912778  0.23325758]
 Rot: [[ 0.99469142  0.06318809  0.08109944]
 [ 0.06430624 -0.99785718 -0.01124784]
 [ 0.08021493  0.01640333 -0.99664255]]
 Qtn: [0.00692215 0.99864798 0.03191674 0.04038319]
 from franka_tool to world
Enter a number: 2
joints: 
[-1.42858148  1.130616    1.65057548 -2.15977689 -1.22175978  1.8022195
  1.44610975]

"""


"""
NEW POSITION 2
pose: 
Tra: [0.50065449 0.12779873 0.29138585]
 Rot: [[ 0.99980783  0.0181655  -0.00592152]
 [ 0.01652556 -0.66663672  0.74519382]
 [ 0.00958931 -0.74514848 -0.66682164]]
 Qtn: [-0.40815116  0.91286173  0.00950063  0.00100448]
 from franka_tool to world
Enter a number: 2
joints: 
[-0.15855773 -0.00895251  0.16191618 -2.276509    0.96872168  2.00932934
  0.21209481]
Enter a number: 1
pose: 
Tra: [0.4873672  0.11905109 0.2836783 ]
 Rot: [[ 0.99891721  0.03604389  0.02908632]
 [ 0.00136984 -0.65070387  0.75932499]
 [ 0.04629561 -0.75846296 -0.65006118]]
 Qtn: [-0.4177775   0.90825138  0.01029829  0.02074919]
 from franka_tool to world
Enter a number: 2
joints: 
[ 1.4778048   0.81882155 -1.7374143  -2.3023647   1.51382283  1.34221706
 -0.04229927]

"""