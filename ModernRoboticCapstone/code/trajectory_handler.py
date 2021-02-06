import numpy as np
import modern_robotics.core as mr

class TrajectoryHandler:
    def __init__(self, T_sc_init, T_sc_goal, T_se_init, T_ce_standoff, T_ce_grasp, robot):
        self.T_grasp = np.dot(T_sc_init, T_ce_grasp)
        self.T_release = np.dot(T_sc_goal, T_ce_grasp)
        self.T_standoff_init = np.dot(T_sc_init, T_ce_standoff)
        self.T_standoff_final = np.dot(T_sc_goal, T_ce_standoff) 
        
        # Define the 8 segment of the overall trajectory
        self.T_path = [T_se_init, self.T_standoff_init, self.T_grasp, self.T_grasp, self.T_standoff_init,
                       self.T_standoff_final, self.T_release, self.T_release, self.T_standoff_final]

        self.ref_traj = []
        self.traj_file = self.states_file = open("../results/trajectory.csv", "w") 
        
        
    
    def compute_segment_times(self, tot_time, t_grasp_release, dt):
        """ 
        Function which computes the duration of each segment depending on its length
        :param: tot_time = duration of the complete trajectory
        :param: t_grasp_release = time that the gripper takes to open/close
        :return: Tf = list of values which represent the duration of each segment
        """
        # Retrieve the position of the points of interests
        p_se_init = np.array([self.T_path[0][0][3], self.T_path[0][1][3], self.T_path[0][2][3]])
        p_standoff_init = np.array([self.T_path[1][0][3], self.T_path[1][1][3], self.T_path[1][2][3]])
        p_grasp = np.array([self.T_path[2][0][3], self.T_path[2][1][3], self.T_path[2][2][3]])
        p_standoff_final = np.array([self.T_path[5][0][3], self.T_path[5][1][3], self.T_path[5][2][3]])
        p_release = np.array([self.T_path[6][0][3], self.T_path[6][1][3], self.T_path[6][2][3]])

        # Compute the lenght of each segment
        d1 = np.linalg.norm(p_standoff_init - p_se_init)    # se_init to standoff_init
        d2 = np.linalg.norm(p_grasp - p_standoff_init) # se_init to grasp
        d4 = d2 # grasp to se_init
        d5 = np.linalg.norm(p_standoff_final - p_standoff_init)  # standoff_init to standoff_final
        d6 = np.linalg.norm(p_release - p_standoff_final)  # standoff_final to release
        d8 = d6  # release to standoff_final

        d_tot = d1 + d2 + d4 + d5 + d6 + d8

        # In advance we know that some time is dedicated to open and close the gripper
        rem_time = tot_time - 2 * t_grasp_release 

        t1 = d1 * rem_time / d_tot
        t2 = d2 * rem_time / d_tot
        t3 = t_grasp_release
        t4 = d4 * rem_time / d_tot
        t5 = d5 * rem_time / d_tot
        t6 = d6 * rem_time / d_tot
        t7 = t_grasp_release
        t8 = d8 * rem_time / d_tot

        Tf = [t1,t2,t3,t4,t5,t6,t7,t8]
        # As requested from the text, each segment time  must be an integer multiple of dt
        for i in range(0, len(Tf)):
            Tf[i] = round((Tf[i]*dt)/dt)
        
        return Tf

    def generate_trajectory(self, k, robot, Tf, dt):
        robot.gripper = 0
        # For each segment, compute the trajectory
        
        for i in range(0, 8):
            if i == 2:
                # Grasping
                robot.gripper = 1
            if i == 6:
                # Releasing
                robot.gripper = 0
            # Number of points in the discrete rapresentation of the i-th trajectory segment
            N = (Tf[i] * k) / dt 
            seg_traj = mr.CartesianTrajectory(Xstart=self.T_path[i], Xend=self.T_path[i + 1], Tf=Tf[i], 
                                              N=N, method=3)
            for j in range(0, len(seg_traj)):
                p = [seg_traj[j][0][0], seg_traj[j][0][1], seg_traj[j][0][2], seg_traj[j][1][0], seg_traj[j][1][1], 
                     seg_traj[j][1][2], seg_traj[j][2][0], seg_traj[j][2][1], seg_traj[j][2][2], seg_traj[j][0][3], 
                     seg_traj[j][1][3], seg_traj[j][2][3], robot.gripper]
                self.ref_traj.append(p)
            self.log_trajectory(traj=seg_traj, gripper=robot.gripper)
        
    
    def log_trajectory(self, traj, gripper):
        # Each row is "r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state"
        for i in range(0, len(traj)):
            row = " %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n" % (traj[i][0][0], traj[i][0][1],
            traj[i][0][2], traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][2][0], traj[i][2][1], traj[i][2][2], 
            traj[i][0][3], traj[i][1][3], traj[i][2][3], gripper)
            self.traj_file.write(row)