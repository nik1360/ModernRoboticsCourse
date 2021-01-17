import csv
import numpy as np
from scipy.optimize import linprog

def init(contacts_filename, bodies_filename):
    cont_file = open(contacts_filename,'r')
    bod_file = open(bodies_filename,'r')
    cont_reader = csv.reader(cont_file, delimiter=',')
    bod_reader = csv.reader(bod_file, delimiter=',')

    n_cont = sum(1 for row in cont_file)
    n_bod = sum(1 for row in bod_file)

    cont_file.seek(0)
    bod_file.seek(0)

    F = np.zeros((3*n_bod, 2*n_cont))
    i_c = 0
    for row in cont_reader:
        b1 = int(row[0]) - 1  # index of the body in wich the contact normal force enters (0 indexing)
        b2 = int(row[1]) - 1
        cp = np.array([float(row[2]), float(row[3])]) # contact point coordinates
        theta = float(row[4]) # angle of the normal contact force
        mu = float(row[5]) # friction coefficient

        alpha = 2*np.arctan(mu) # friction cone angle
        f1 = np.array([np.cos(theta - 0.5*alpha), np.sin(theta - 0.5*alpha)])
        f2 = np.array([np.cos(theta + 0.5*alpha), np.sin(theta + 0.5*alpha)])
        m1 = np.cross(cp, f1)
        m2 = np.cross(cp, f2)

        # Add the contact wrenches to the matrix F
        # consider cone edge 1
        if b1 >= 0:  # ground is not considered as a body
            F[b1*3 + 0][i_c] = m1
            F[b1*3 + 1][i_c] = f1[0]
            F[b1*3 + 2][i_c] = f1[1]
        if b2 >=0 :
            F[b2*3 + 0][i_c] = -m1
            F[b2*3 + 1][i_c] = -f1[0]
            F[b2*3 + 2][i_c] = -f1[1]
        i_c += 1

        # consider cone edge 2
        if b1 >= 0:
            F[b1*3 + 0][i_c] = m2
            F[b1*3 + 1][i_c] = f2[0]
            F[b1*3 + 2][i_c] = f2[1]
        if b2 >= 0:
            F[b2*3 + 0][i_c] = -m2
            F[b2*3 + 1][i_c] = -f2[0]
            F[b2*3 + 2][i_c] = -f2[1]
        i_c += 1

    F_ext = np.zeros(3*n_bod)
    a_g = np.array([0, -9.8])
    i_b = 0 
    for row in bod_reader:
        cm = np.array([float(row[0]), float(row[1])])
        mass = float(row[2])

        f_g = mass * a_g # gravitational force
        m = np.cross(cm, f_g) # moment
        F_ext[i_b*3 + 0] = m
        F_ext[i_b*3 + 1] = f_g[0]
        F_ext[i_b*3 + 2] = f_g[1] 
        i_b += 1

    return F, F_ext

def solve_lp(F, F_ext):
    c = np.ones(F.shape[1])
    
    A = - np.identity(F.shape[1])
    b = - np.zeros(F.shape[1])
    A_eq = F
    b_eq = - F_ext
    
    res = linprog(c=c, A_eq=A_eq, b_eq=b_eq, A_ub=A, b_ub=b)
    return res['success'], res['x']



if __name__=="__main__":
    print("---------------------Case 1----------------------------")
    F, F_ext = init('../results/case1_contacts.csv', '../results/case1_bodies.csv')
    stand, k = solve_lp(F, F_ext)
    print("Assembly will stand! k=", k) if stand else print("Assembly will NOT stand")
    print("---------------------Case 2----------------------------")
    F, F_ext = init('../results/case2_contacts.csv', '../results/case2_bodies.csv')
    stand, k = solve_lp(F, F_ext)
    print("Assembly will stand! k=", k) if stand else print("Assembly will NOT stand")
