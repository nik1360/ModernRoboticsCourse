import numpy as np
from scipy.optimize import linprog


def evaluate_form_closure(F):
    # Function which solves the LP problem 
        
    # find       k
    # minimizing 1^T * k
    # such that  Fk = 0
    #            k_i >= 1 for every i=1...j

    # If there exist asolution, then there is form closure!
    
    c = np.ones(F.shape[1])
    
    A = - np.identity(F.shape[1])
    b = - np.ones(F.shape[1])
    A_eq = F
    b_eq = np.zeros(F.shape[0])
    
    res = linprog(c=c, A_eq=A_eq, b_eq=b_eq, A_ub=A, b_ub=b)
    return res['success']
    

if __name__=='__main__':
    # Test the form closure of the two cases presented in the screenshots
    # contained i nthe folder "results"
    print("Considering Case 1 ...")
    F_test_1 = np.array([[2, -3, -2, 3],[0,1,0,-1],[1,0,-1,0]])
    closure = evaluate_form_closure(F=F_test_1)
    print("Form closure!") if closure else print("NOT in form closure")
    
    print("Considering Case 2 ...")
    F_test_2 = np.array([[2, -3],[0,1],[1,0]])
    closure = evaluate_form_closure(F=F_test_2)
    print("Form closure!") if closure else print("NOT in form closure")
    
