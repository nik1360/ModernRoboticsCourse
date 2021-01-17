import modern_robotics.core as mr
from inverse_kinematics import IKinBodyIterate
import numpy as np

# define the constants of the robot 

l1 = 0.425 # [m]
l2 = 0.392
h1 = 0.089
h2 = 0.095
w1 = 0.109
w2 = 0.082

# define the parameters for the algorithm
eomg = 0.001 # [rad]
ev = 0.0001  # [m]

# define matrix M and the list od screw axis in the {b} frame
M = np.array([[-1, 0, 0, l1+l2], [0, 0, 1, w1+w2], [0, 1, 0, h1-h2], [0, 0, 0, 1]])
B = np.array([[0,0,0,0,0,0],
	      [1,0,0,0,-1,0],
	      [0,1,1,1,0,1],
              [w1+w2, h2, h2, h2, -w2, 0],
	      [0, -l1-l2, -l2, 0,0,0],
	      [l1+l2, 0,0,0,0,0]])

# define the desired frame for the end effector
Tsd = np.array([[0,1,0,-0.5],[0,0,-1,0.1],[-1,0,0,0.1],[0,0,0,1]])

# initial guess
thetalist0 = np.array([np.pi/2, np.pi/3, -np.pi/2, np.pi, np.pi/6, np.pi/2])

theta, convergence = IKinBodyIterate(B, M, Tsd, thetalist0, eomg, ev)


