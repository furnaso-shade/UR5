
import numpy as np

#suppresing scientific notations in the matrices
np.set_printoptions(suppress=True)

#using ur5 data
a3, a4 = 425, 392
d1, d4, d5, d6 = 0.0891, 0.109, 0.0946, 0.0823
t1, t2, t3, t4, t5, t6 = 0, 90, 90, 0, 0, 0

#define dh table
dhtable = np.array([
    [0, 0, d1, t1],
    [90, 0, 0, t2],
    [0, a3, 0, t3],
    [0, a4, d4, t4],
    [90, 0, d5, t5],
    [-90, 0, d6, t6]
    ])
    
#calculating transformation matrix (T) with data drawn from dh table
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
    
    print(f"-transformation matrix for joint {i+1}")
    print(np.round(T, 4))
    print()


