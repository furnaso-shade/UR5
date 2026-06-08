#JACO
import numpy as np

#suppressing the scientific notations in the matrix
np.set_printoptions(suppress=True)

#using ur5 data
a2, a3 = 0.425, 0.392
d1, d4, d5, d6 = 0.0891, 0.109, 0.0946, 0.0823

def matrices(q):
    t1, t2, t3, t4, t5, t6 = q[0], q[1], q[2], q[3], q[4], q[5]
    dhtable = np.array([
    [90, 0, d1, t1],
    [0, a2, 0, t2],
    [0, a3, 0, t3],
    [90, 0, d4, t4],
    [-90, 0, d5, t5],
    [0, 0, d6, t6]
    ])
    
    #using an identity matrix as final matrix initially
    Tfinal = np.eye(4)
    T_i = [np.eye(4)]
    
    #calculating end effector transformation matrix (Tfinal) with data drawn from dh table
    for i, row in enumerate(dhtable):
        alphadeg = row[0]
        a = row[1]
        d = row[2]
        thetadeg = row[3]
        
        alpha = np.radians(alphadeg)
        theta = np.radians(thetadeg)
        
        ca = np.cos(alpha)
        ct = np.cos(theta)
        sa = np.sin(alpha)
        st = np.sin(theta)
        
        T = np.array([
            [ct,       -st,       0,   a],
            [st * ca,  ct * ca, -sa, -sa * d],
            [st * sa,  ct * sa,  ca,  ca * d],
            [0,        0,        0,   1]
        ])
        
        Tfinal = Tfinal @ T
        T_i.append(Tfinal)
    return T_i
        
#creating jacobian matrix
def jacobian(q):
    T_i = matrices(q)
    J = np.zeros((6,6))
    pEnd = T_i[-1][0:3, 3]
    for i in range(6):
        p_i = T_i[i][0:3, 3]
        z_i = T_i[i][0:3, 2]
        J[0:3, i] = np.cross(z_i, (pEnd - p_i))
        J[3:6, i] = z_i
    return J
    
qTest = [0, 90, 90, 0, 0, 0]
J = jacobian(qTest)
print (J)
    