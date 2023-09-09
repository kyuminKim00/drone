import numpy as np
import scipy.sparse.linalg as sla
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib.axes import Axes




def trajectory_gen(x_0, wp, tk):
    T = tk[-1] # Total time = the time when the drone pass the final waypoint
    delt = 0.1 # delta t. time interval for discrete time system [sec]
    n = int(T/delt) # number of timesteps. if total time is 10sec with delta_t of 0.1sec, than the number of total timesteps is 10/0.1 = 100 steps

    gamma = .05 # drag coefficient.


### Step.1 : define all coefficient matrices. A,B,C,G and H ###

    A = np.zeros((6,6))
    B = np.zeros((6,3))

    A[0,0] = 1
    A[1,1] = 1
    A[2,2] = 1
    A[0,3] = (1-gamma*delt/2)*delt
    A[1,4] = (1-gamma*delt/2)*delt
    A[2,5] = (1-gamma*delt/2)*delt
    A[3,3] = 1 - gamma*delt
    A[4,4] = 1 - gamma*delt
    A[5,5] = 1 - gamma*delt

    B[0,0] = delt**2/2
    B[1,1] = delt**2/2
    B[2,2] = delt**2/2
    B[3,0] = delt
    B[4,1] = delt
    B[5,2] = delt

    C = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]])

    tk = (tk/delt - 1).astype(int)
    K = len(wp[0])


    G = np.zeros( (3*n,6) )

    for i in range(n):
        G[3*i:3*(i+1),:] = C@np.linalg.matrix_power(A,i+1)

    H = np.zeros( (3*n,3*n) )
    H_first = np.zeros( (3*n,3) )
    
    for i in range(n):
        H_first[3*i:3*(i+1),:] = C@np.linalg.matrix_power(A,i)@B
    for i in range(n):
        H[3*i:,3*i:3*(i+1)] = H_first[:3*(n-i),:]

    S = np.zeros( (3*K,3*n))

    for k in range(K):
        S[3*k:3*k+3,3*tk[k]:3*tk[k]+3] = np.eye(3)


### Step.2 : solve optimization problem ###

    u_hat = sla.lsqr(S@H,wp.T.flatten() - S@G@x_0)[0]

    u_vec = u_hat

    u_opt = u_vec.reshape(n,3).T

    x = np.zeros((6,n+1))

    x[:,0] = x_0

    for t in range(n):
        x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])

    return x




### Example ###


# initial state : stopped at (local_x,local_y,local_z)
x_0 = np.array([6.0,6.0,2.0,0.0,0.0,0.0])


# waypoint input : wp1 = (6,6,1), wp2 = (0,0,1), wp3 = (2,2,1), wp4 = (0,0,1)
# wp = np.array([[ 10+x_0[0], 20+x_0[0], 40+x_0[0], 50+x_0[0]],
#                 [ 5+x_0[1], -10+x_0[1], 10+x_0[1], 10+x_0[1]],
#                 [ 6+x_0[2], 7+x_0[2], 9+x_0[2], 10+x_0[2]]])

# wp = np.array([[ 1+x_0[0], 1+x_0[0], 2+x_0[0], 3+x_0[0]],
#                 [ 1+x_0[1], 2+x_0[1], 2+x_0[1], 3+x_0[1]],
#                 [ 1+x_0[2], 1+x_0[2], 1+x_0[2], 3+x_0[2]]])

# wp = np.array([[ 6+x_0[0], 3+x_0[0], 1.5+x_0[0], 0+x_0[0]],
#                 [ 6+x_0[1], 3+x_0[1], 1.5+x_0[1], 0+x_0[1]],
#                 [ 2+x_0[2], 2+x_0[2], 2+x_0[2], 2+x_0[2]]])

wp = np.array([[ 6,  0],
                [ 6, 0],
                [ 2, 2]])
# wp = np.array([[ 10+x_0[0], x_0[0]],
#                 [ 10+x_0[1], x_0[1]],
#                 [ 10+x_0[2],  1+x_0[2]]])


# pass the wp1 at 10sec, wp2 at 15sec, wp3 at 20sec, wp4 at 30sec
tk = np.array([1,20])    



# Calculate the trajectory
x = trajectory_gen(x_0,wp,tk)

traj =x
print(traj[3:])


### visualization ###
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
# ax.set_xlim(0,50)
# ax.set_ylim(-25,25)
# ax.set_zlim(0,20)


ax.plot(traj[0],traj[1],traj[2],'b--',label="Trajectory")
# ax.scatter(wp[0,0],wp[1,0],wp[2,0],c="black")
# ax.scatter(wp[0,1],wp[1,1],wp[2,1],c="black")
# ax.scatter(wp[0,2],wp[1,2],wp[2,2],c="black")
# ax.scatter(wp[0,3],wp[1,3],wp[2,3],c="black")

ax.set_xlabel("north")
ax.set_ylabel("east")
ax.set_zlabel("up")

ax.legend()


plt.show()