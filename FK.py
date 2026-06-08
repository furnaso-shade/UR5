#FK
import numpy as np

#suppressing the scientific notations in the matrix
np.set_printoptions(suppress=True)

#using ur5 data
a2, a3 = 0.425, 0.392
d1, d4, d5, d6 = 0.0891, 0.109, 0.0946, 0.0823

#incorporating joint vector using test values
q = [0, 90, 90, 0, 0, 0]
t1, t2, t3, t4, t5, t6 = q[0], q[1], q[2], q[3], q[4], q[5]

#define dh table
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
#configuring position and orientation matrices
position = Tfinal[0:3, 3]
orientation = Tfinal[0:3, 0:3]


print("---end effector transformation matrix---")
print(np.round(Tfinal, 4))

    
   


