import math
import numpy as np
import modern_robotics as mr

import csv

# --------------------------------------------- ROBOT PARAMETERS -------------------------------------------

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]


# --------------------------------------------- SIMULATION PARAMETERS -------------------------------------------

tau = np.array([0,0,0,0,0,0])  # no joint torques applied
g = np.array([0,0,-9.8])  # gravity force toward -z axis
dthetalist = np.array([0,0,0,0,0,0])  # zero velocity for every joint 
Ftip = np.array([0,0,0,0,0,0])  # no wrench applied to the end effector

dt = 0.01 # [s]

# ------------------------------------------------- CREATE CSV FOR SIMULATION 1 ----------------------------------------------------
max_time = 3   
sim_time = 0
thetalist = np.array([0,0,0,0,0,0])   # initial joint vector [rad]
print("creating simulation1.csv, please wait...")
file1 = open("simulation1.csv", "w", newline="") 
while sim_time < max_time:
	
    csv.writer(file1).writerow([thetalist[0], thetalist[1], thetalist[2], thetalist[3], thetalist[4], thetalist[5]]) # update the csv

    ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, tau, g, Ftip, Mlist, Glist, Slist)  # retrieve joint accelerations
    dthetalistNext = dthetalist + ddthetalist * dt
    thetalistNext = thetalist + dthetalistNext * dt

    # thetalistNext, dthetalistNext = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)  # compute the next joint position and velocity

    # update 
    thetalist = thetalistNext
    dthetalist = dthetalistNext

    sim_time += dt

print("simulation1.csv created!")
# --------------------------------------------------CREATE CSV FOR SIMULATION 2 ------------------------------------------------------
max_time = 5  
sim_time = 0
thetalist = np.array([0,-1,0,0,0,0])  # initial joint vector [rad]
print("creating simulation2.csv, please wait...")
file2 = open("simulation2.csv", "w", newline="") 
while sim_time < max_time:
	
    csv.writer(file2).writerow([thetalist[0], thetalist[1], thetalist[2], thetalist[3], thetalist[4], thetalist[5]])  # update the csv

    ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, tau, g, Ftip, Mlist, Glist, Slist)
    dthetalistNext = dthetalist + ddthetalist * dt
    thetalistNext = thetalist + dthetalistNext * dt

    # thetalistNext, dthetalistNext = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)

    # update 
    thetalist = thetalistNext
    dthetalist = dthetalistNext

    sim_time += dt

print("simulation2.csv created!")

