from scipy import linalg as la
import matplotlib.pyplot as pl
import matplotlib.patches as pa
import numpy as np

import quadrotor as quad
import formation_distance as form
import quadlog
import animation as ani

from guidance_vector_field import GuidanceVectorFieldEllipse

# Quadrotor number 1
m = 1 # Kg
l = 1 # m
Jxx = 7.5e-3 # Kg/m^2
Jyy = Jxx
Jzz = 1.3e-2
Jxy = 0
Jxz = 0
Jyz = 0
J = np.array([[Jxx, Jxy, Jxz], \
              [Jxy, Jyy, Jyz], \
              [Jxz, Jyz, Jzz]])
CDl = 9e-3
CDr = 9e-4
kt = 3.13e-5  # Ns^2
km = 7.5e-7   # Ns^2
kw = 1/0.18   # rad/s

# Initial conditions
att_0 = np.array([0.0, 0.0, 0.0])
pqr_0 = np.array([0.0, 0.0, 0.0])
xyz_0 = np.array([0.0, 0.0, 0.0])
v_ned_0 = np.array([0.0, 0.0, 0.0])
w_0 = np.array([0.0, 0.0, 0.0, 0.0])

# Setting quads
q1 = quad.quadrotor(1, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz_0, v_ned_0, w_0)

	#Setting second quad

q2 = quad.quadrotor(2, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz_0, v_ned_0, w_0)



# Simulation parameters
tf = 200
dt = 1e-2
time = np.linspace(0, tf, tf/dt)
it = 0
frames = 100

# Data log
q1_log = quadlog.quadlog(time)
q2_log = quadlog.quadlog(time)

# Plots
quadcolor = ['k','b'] #Array for number of quads
pl.close("all")
pl.ion()
fig = pl.figure(0)
axis3d = fig.add_subplot(131, projection='3d')
axis2d = fig.add_subplot(132)
axis_gfv = fig.add_subplot(133)
pl.figure(0)

# Desired position and heading
xyz_d_q1 = np.array([5, 3, -10])
xyz_d_q2 = np.array([0, 0, -6])


q1.yaw_d = -np.pi/4
q2.yaw_d = np.pi/4

#### not sure from here:

# Formation Control
# Shape
side = 5
Btriang = np.array([[-1, 0],[0, -1]])
dtriang = np.array([side, side, side])

# Motion
mu = 0e-2*np.array([1, 1])
tilde_mu = 0e-2*np.array([1, 1])

# Formation
fc = form.formation_distance(2, 1, dtriang, mu, tilde_mu, Btriang, 5e-2, 5e-1)


# Guidance vector field
BOUNDARY = 50

gvf = GuidanceVectorFieldEllipse(15,10,0,0,0.55) 
axis_gfv = gvf.plotIt(axis_gfv, BOUNDARY)

# Ellipse
e1 = pa.Ellipse((gvf.x0, gvf.y0), gvf.a*2., gvf.b*2., linewidth=1, fill=False)


CoM_x = np.array([])
CoM_y = np.array([])


ellips_x = np.array([]) 
ellips_y = np.array([])




N = 400
x1 = np.linspace(-BOUNDARY, BOUNDARY, N, endpoint=True)
y1 = np.linspace(-BOUNDARY, BOUNDARY, N, endpoint=True)
for x in x1:
    for y in y1:
        if abs(gvf.computeError((x,y))) < 0.1:
            ellips_x = np.concatenate([ellips_x, [x]])
            ellips_y = np.concatenate([ellips_y, [y]])
            
arrows = gvf.plotIt(axis2d, BOUNDARY)

for t in time:

    # Simulation

    if t < 45:
        q1.set_xyz_ned_lya(xyz_d_q1)
        q2.set_xyz_ned_lya(xyz_d_q2)

    elif t >= 45 and t < 46:
        X = np.append(q1.xyz[0:2], q2.xyz[0:2])
        V = np.append(q1.v_ned[0:2], q2.v_ned[0:2])
        U = fc.u_acc(X, V)


        q1.set_a_2D_alt_lya(U[0:2], q2.xyz[2])
        q2.set_a_2D_alt_lya(U[2:4], q1.xyz[2])

    else:
        #calculate center of mass: 
        CoM = q1.xyz[0:2] + np.multiply(np.subtract(q2.xyz[0:2],q1.xyz[0:2]), 0.5)

        CoM_x = np.concatenate([CoM_x,[CoM[0]]])
        CoM_y = np.concatenate([CoM_y,[CoM[1]]])

        CoM_d_vel = gvf.getDesiredVelocity(CoM)

        q1.set_v_2D_alt_lya(CoM_d_vel, q2.xyz[2])
        q2.set_v_2D_alt_lya(CoM_d_vel, q1.xyz[2])


        # dvelQ1 = gvf.getDesiredVelocity(q1.xyz)
        # dvelQ2 = gvf.getDesiredVelocity(q2.xyz)
        # q1.set_v_2D_alt_lya(dvelQ1, q2.xyz[2])
        # q2.set_v_2D_alt_lya(dvelQ2, q1.xyz[2])

        # try to let them both follow a guidance vector field



        #q1.set_a_2D_alt_lya(np.array([0,0]), q2.xyz[2])
        #q2.set_a_2D_alt_lya(np.array([0,0]), q1.xyz[2])


    q1.step(dt)
    q2.step(dt)

    # Animation
    if it%frames == 0:


    	# Print outs during animation

    	#print q1.xyz

            

        axis3d.cla()
        ani.draw3d(axis3d, q1.xyz, q1.Rot_bn(), quadcolor[0])
        ani.draw3d(axis3d, q2.xyz, q2.Rot_bn(), quadcolor[1])
        axis3d.set_xlim(-BOUNDARY, BOUNDARY)
        axis3d.set_ylim(-BOUNDARY, BOUNDARY)
        axis3d.set_zlim(0, 15)
        axis3d.set_xlabel('South [m]')
        axis3d.set_ylabel('East [m]')
        axis3d.set_zlabel('Up [m]')
        axis3d.set_title("Time %.3f s" %t)


        axis2d.cla()
        axis2d.add_patch(e1)
        if t >= 45:
            axis2d = arrows
        axis2d.plot(q1.xyz[0], q1.xyz[1], 'bo')
        axis2d.plot(q2.xyz[0], q2.xyz[1], 'bo')
        axis2d.plot(CoM_x, CoM_y, 'bo', color="orange")
        #axis2d.plot(ellips_x, ellips_y, 'bo', color="green")
        axis2d.set_xlabel('x')
        axis2d.set_ylabel('y')
        axis2d.set_xlim(-BOUNDARY, BOUNDARY)
        axis2d.set_ylim(-BOUNDARY, BOUNDARY)
        axis2d.axis('equal')


        

        pl.pause(0.001)
        pl.draw()
        

    # Log
    q1_log.xyz_h[it, :] = q1.xyz
    q1_log.att_h[it, :] = q1.att
    q1_log.w_h[it, :] = q1.w
    q1_log.v_ned_h[it, :] = q1.v_ned
    q1_log.xi_g_h[it] = q1.xi_g
    q1_log.xi_CD_h[it] = q1.xi_CD

    q2_log.xyz_h[it, :] = q2.xyz
    q2_log.att_h[it, :] = q2.att
    q2_log.w_h[it, :] = q2.w
    q2_log.v_ned_h[it, :] = q2.v_ned
    q2_log.xi_g_h[it] = q2.xi_g
    q2_log.xi_CD_h[it] = q2.xi_CD


    it+=1
    
    # Stop if crash
    if (q1.crashed == 1 or q2.crashed == 1):
        break

pl.figure(1)
pl.plot(time, q1_log.w_h[:, 0], label="w_1")
pl.plot(time, q1_log.w_h[:, 1], label="w_2")
pl.plot(time, q1_log.w_h[:, 2], label="w_3")
pl.plot(time, q1_log.w_h[:, 3], label="w_4")
pl.xlabel("Time [s]")
pl.ylabel("Motor angular velocity [rad/s]")
pl.grid()
pl.legend()

pl.figure(2)
pl.plot(time, q1_log.att_h[:, 0], label="roll")
pl.plot(time, q1_log.att_h[:, 1], label="pitch")
pl.plot(time, q1_log.att_h[:, 2], label="yaw")
pl.xlabel("Time [s]")
pl.ylabel("Attitude angle [rad]")
pl.grid()
pl.legend()

pl.figure(3)
pl.plot(time, -q1_log.xyz_h[:, 2], label="UP")
pl.plot(time, q1_log.xyz_h[:, 0], label="X")
pl.plot(time, q1_log.xyz_h[:, 1], label="Y")
pl.xlabel("Time [s]")
pl.ylabel("Position [m]")
pl.grid()
pl.legend()

pl.figure(4)
pl.plot(time, -q1_log.v_ned_h[:, 2], label="-V_z")
pl.plot(time, q1_log.v_ned_h[:, 0], label="V_x")
pl.plot(time, q1_log.v_ned_h[:, 1], label="V_y")
pl.xlabel("Time [s]")
pl.ylabel("Velocity [m/s]")
pl.grid()
pl.legend()

pl.figure(5)
pl.plot(time, q1_log.xi_g_h, label="${\\xi}_g$")
pl.plot(time, q1_log.xi_CD_h, label="${\\xi}_{CD}$")
pl.xlabel("Time [s]")
pl.ylabel("Estimators value")
pl.grid()
pl.legend()

pl.pause(0)
