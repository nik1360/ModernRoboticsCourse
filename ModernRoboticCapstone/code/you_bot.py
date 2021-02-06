import numpy as np
import modern_robotics.core as mr

class YouBot:
    def __init__(self, l ,w ,r, wheels_limit, joints_limit, chassis_init, manipulator_init):
        self.l = l
        self.w = w
        self.r = r
        self.m_q = manipulator_init  # Manipulator configuration
        self.c_q = chassis_init  # Chassis configuration
        self.w_q = np.zeros(4)  # Wheel angles
        self.gripper = 0  # 0 = gripper open, 1 = gripper closed

        self.joints_limit = joints_limit 
        self.wheels_limit = wheels_limit

        self.F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                                 [1, 1, 1, 1],
                                 [-1, 1, -1, 1]])
        self.F_6 = np.array([[0]*4,[0]*4])
        self.F_6 = np.append(self.F_6,self.F,axis=0)
        self.F_6 = np.append(self.F_6,np.array([[0]*4]),axis=0)

        # Definition of some Transformation matrices associated to the robot

        # Fixed offset from the chassis frame {b} to the base frame of the arm {0}
        self.T_b0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
        # When the arm is at its home configuration, the end-effector frame {e} relative 
        # to the arm base frame {0}
        self.M_0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
        # When the arm is at its home configuration, the screw axes \mathcal{B} for the 
        # five joints are expressed in the end-effector frame {e}

        self.Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                          [0, -1, 0, -0.5076, 0, 0],
                          [0,-1, 0, -0.3526, 0, 0],
                          [0, -1, 0, -0.2176, 0, 0],
                          [0, 0, 1, 0, 0, 0]]).T

        # Open the output files
        self.states_file = open("../results/kinematics.csv", "w") 
    
    def nextStep(self, u, dt):
        """
        Perform a simulation step given a certain control action:
        :param: u = [u_w1...u_w4 u_j1..u_j5] is the control signal for joints and wheels velocity
        :param: dt = simulation timestep 
        """
        u_wheels = np.array([u[0], u[1], u[2], u[3]])
        u_joints = np.array([u[4], u[5], u[6], u[7], u[8]])
        # self.limit_control(u_wheels=u_wheels, u_joints=u_joints)
        
        
        # Update joint and wheel poistions
        new_m_q = self.m_q + (u_joints * dt)
        new_w_q = self.w_q + (u_wheels * dt)  

        # Use odometry to update the chassis configuration
        delta_wheels = (u_wheels * dt)
        V_b = np.dot(self.F, delta_wheels) 

        w_bz = V_b[0]
        v_bx = V_b[1]
        v_by = V_b[2]

        if w_bz == 0:
            delta_c_q_b = np.array([0, v_bx, v_by])
        else:
            delta_c_q_b = np.array([w_bz,
                                   (v_bx*np.sin(w_bz) + v_by*(np.cos(w_bz) - 1))/ w_bz,
                                   (v_by*np.sin(w_bz) + v_bx*(1 - np.cos(w_bz))) / w_bz])
        delta_c_q = np.dot(np.array([[1,0,0],
                                     [0, np.cos(self.c_q[0]), -np.sin(self.c_q[0])],
                                     [0, np.sin(self.c_q[0]), np.cos(self.c_q[0])]]), delta_c_q_b)
        
        self.c_q = self.c_q + delta_c_q

        self.m_q = new_m_q
        self.w_q = new_w_q

        self.log_state()
    
    def limit_control(self, u_joints, u_wheels):
        # Limit the input control signals

        for i in range(0, len(u_joints)):
            if u_joints[i] > self.joints_limit[i]:
                u_joints[i] = self.joints_limit[i]
            if u_joints[i] < - self.joints_limit[i]:
                u_joints[i] = - self.joints_limit[i]

        for i in range(0, len(u_wheels)):
            if u_wheels[i] > self.wheels_limit[i]:
                u_wheels[i] = self.wheels_limit[i]
            if u_wheels[i] < - self.wheels_limit[i]:
                u_wheels[i] = - self.wheels_limit[i]

    def log_state(self):
        row = " %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n" % (self.c_q[0], self.c_q[1],
        self.c_q[2], self.m_q[0], self.m_q[1], self.m_q[2], self.m_q[3], self.m_q[4], self.w_q[0], self.w_q[1],
        self.w_q[2], self.w_q[3], self.gripper)
        
        self.states_file.write(row) 
    
    def forward_kinematics(self):
        T_0e = mr.FKinBody(M=self.M_0e, Blist=self.Blist, thetalist=self.m_q)

        T_sb = np.array([[np.cos(self.c_q[0]), -np.sin(self.c_q[0]), 0, self.c_q[1]],
                        [np.sin(self.c_q[0]), np.cos(self.c_q[0]), 0, self.c_q[2]],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])
        T_se = np.linalg.multi_dot((T_sb, self.T_b0, T_0e))

        return T_se

    def compute_J_e(self):
        T_0e = mr.FKinBody(M=self.M_0e, Blist=self.Blist, thetalist=self.m_q)
        
        J_base = np.dot(mr.Adjoint(np.dot(mr.TransInv(T_0e), mr.TransInv(self.T_b0))), self.F_6)
        J_arm = mr.JacobianBody(Blist=self.Blist, thetalist=self.m_q)
        
        J_e = np.append(J_base, J_arm, axis=1)

        return J_e

        
        


        