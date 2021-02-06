from you_bot import YouBot
from trajectory_handler import TrajectoryHandler
import numpy as np
import modern_robotics.core as mr
import matplotlib.pyplot as plt


def plot_err(X_err_list):
    err1 = []
    err2 = []
    err3 = []
    err4 = []
    err5 = []
    err6 = []
    for i in range(0, len(X_err_list)):
        err1.append(X_err_list[i][0])
        err2.append(X_err_list[i][1])
        err3.append(X_err_list[i][2])
        err4.append(X_err_list[i][3])
        err5.append(X_err_list[i][4])
        err6.append(X_err_list[i][5])
    
    x = np.linspace(0, 16, 1000)
    plt.grid(True)
    plt.plot(err1, label='err1')
    plt.plot(err2, label='err2')
    plt.plot(err3, label='err3')
    plt.plot(err4, label='err4')
    plt.plot(err5, label='err5')
    plt.plot(err6, label='err6')

    plt.legend(loc="upper right")
    
    plt.savefig('../results/err_plot.png')



if __name__ == '__main__':
    # Simulation parameters
    timestep = 0.01
    sim_duration = 16 # [s]
    
    sim_steps = int(sim_duration / timestep)

    # Robot Parameter definition
    l = 0.47 / 2
    w = 0.3 / 2
    r = 0.0475
    joints_limit = np.array([10,10,10,10,10])
    wheels_limit = np.array([10,10,10,10])
    chassis_init = np.array([np.pi/4, -0.3, 0.2])  # [phi, x, y]
    manipulator_init = np.array([0, 0, 0.2, -1.6, 0])

    robot = YouBot(l=l, w=w, r=r, joints_limit=joints_limit, wheels_limit=wheels_limit, 
                   chassis_init=chassis_init, manipulator_init=manipulator_init)
    print("YouBot initialized!")

    # Trajectory handler definition
    T_se_init = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]) # Initial config of the manipulator (wiki)
    T_sc_init = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])  # Initial config of the cube (wiki)
    T_sc_goal = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025], [0,0,0,1]]) # Final config of the cube (wiki)

    alpha = np.pi/5
    T_ce_standoff = np.array([[-np.sin(alpha), 0, np.cos(alpha), 0], # Standoff configuration between ee and cube
                             [0, 1, 0, 0],
                             [-np.cos(alpha), 0, -np.sin(alpha), 0.25],
                             [0, 0, 0, 1]])
    T_ce_grasp = np.array([[-np.sin(alpha), 0, np.cos(alpha), 0], # Configuration between ee and cube during grasp
                           [0, 1, 0, 0],
                           [-np.cos(alpha), 0, -np.sin(alpha), 0],
                           [0, 0, 0, 1]])


    th = TrajectoryHandler(T_sc_init=T_sc_init, T_sc_goal=T_sc_goal, T_se_init=T_se_init, 
                           T_ce_standoff=T_ce_standoff, T_ce_grasp=T_ce_grasp, robot=robot)

    Tf = th.compute_segment_times(tot_time=sim_duration, t_grasp_release=0.625, dt=timestep)
    th.generate_trajectory(k=1, dt=timestep, Tf=Tf, robot=robot)
    print("trajectory.csv generated inside the results folder!")

    K_p = np.identity(6) * 5
    K_i = np.identity(6) * 10


    # Main loop
    integral = np.zeros(6)
    X_err_list = []  # error list for the logs
    err_file = open("./results/X_err_log.csv", 'w')

    for i in range(0,sim_steps-1):
        X = robot.forward_kinematics()  # Current actual configuration
        # Desired position a the current instant
        X_d = np.array([[th.ref_traj[i][0], th.ref_traj[i][1], th.ref_traj[i][2], th.ref_traj[i][9]],
                        [th.ref_traj[i][3], th.ref_traj[i][4], th.ref_traj[i][5], th.ref_traj[i][10]],
                        [th.ref_traj[i][6], th.ref_traj[i][7], th.ref_traj[i][8], th.ref_traj[i][11]],
                        [0, 0, 0, 1]])
        robot.gripper = th.ref_traj[i][12]

        # Desired position at the next instant
        X_d_next = np.array([[th.ref_traj[i+1][0], th.ref_traj[i+1][1], th.ref_traj[i+1][2], th.ref_traj[i+1][9]],
                        [th.ref_traj[i+1][3], th.ref_traj[i+1][4], th.ref_traj[i+1][5], th.ref_traj[i+1][10]],
                        [th.ref_traj[i+1][6], th.ref_traj[i+1][7], th.ref_traj[i+1][8], th.ref_traj[i+1][11]],
                        [0, 0, 0, 1]])
        
        # The error twist Xerr that takes X to Xd in unit time is 
        # extracted from the 4x4 se(3) matrix [Xerr] = log(X^(−1) Xd)
        X_err = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X), X_d)))

        # Maintain an estimate of the integral of the error
        integral = integral + (X_err * timestep)

        # Compute the twist which is necessary to go from X_d to X_d_next
        V_d =  mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X_d), X_d_next))/timestep)
        
        # Build the contro law
        term1 = np.dot(mr.Adjoint(np.dot(mr.TransInv(X), X_d)), V_d)
        term2 = np.dot(K_p, X_err)
        term3 = np.dot(K_i, integral)
        V = term1 + term2 + term3

        # Compute the Jacobian and, using the pseudoinverse, obtain the control signal 
        J_e = robot.compute_J_e()
        u = np.dot(np.linalg.pinv(J_e, rcond=1e-3), V)

        # Update the robot state 
        robot.nextStep(u=u, dt=timestep)

        # Log on csv and Store the error
        
        row = "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n"%(X_err[0], X_err[1], X_err[2], X_err[3], X_err[4], X_err[5])
        err_file.write(row)
        X_err_list.append(X_err)
    
    print("kinematics.csv generated inside the results folder!")
    plot_err(X_err_list)
    print("err_plot.png generated in the results folder!")

