import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from MPC.QP_solver import QPController
import spatialgeometry as sg
from MPC.LMPC_solver import LinearMPCController
import time
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Init env
env = swift.Swift()
env.launch(realtime=True)
panda = rtb.models.Panda()
panda.q = panda.qr
env.add(panda)
dt = 0.05

# Init desired position
T_ini = panda.fkine(panda.q)
T_des = panda.fkine(panda.q)
target = sg.Sphere(radius=0.02, pose=T_des, color=[0,1,0])
env.add(target)
env.set_camera_pose([1.0, 1.0, 0.7], [0, 0, 0.4])

# Init LMPC solver for path planning, gamma the gain of the controller as it ensures slows commands u, the lower the faster
lmpc_solver = LinearMPCController(horizon=25, dt=dt, gamma = 0.001,
                                    u_min=np.array([-0.5, -0.5, -0.5, -1.0, -1.0, -1.0]),
                                    u_max=np.array([ 0.5,  0.5,  0.5,  1.0,  1.0,  1.0]))

# Init QP solver for IK with safety
qp_solver = QPController(panda)
qp_solver.solve(np.zeros((6,1)))

# Add sliders to control desired position
x, y, z = 0.0, 0.0, 0.0
def set_x(x_set):
    global T_des, target, x, y, z
    x, T_des = float(x_set), T_ini * sm.SE3.Trans(float(x_set), y, z)
    target.T = T_des
env.add(swift.Slider(lambda x: set_x(x),min=-0.4,max=0.4,step=0.01,desc="x",))
def set_y(y_set):
    global T_des, target, x, y, z
    y, T_des = float(y_set), T_ini * sm.SE3.Trans(x, float(y_set), z)
    target.T = T_des
env.add(swift.Slider(lambda x: set_y(x),min=-0.4,max=0.4,step=0.01,desc="y",))
def set_z(z_set):
    global T_des, target, x, y, z
    z, T_des = float(z_set), T_ini * sm.SE3.Trans(x, y, float(z_set))
    target.T = T_des
env.add(swift.Slider(lambda x: set_z(-x),min=-0.5,max=0.5,step=0.01,desc="z",))

# Loop
while True:
    #Compute desired velocity from simple prop controller
    T_current = panda.fkine(panda.q)
    Uopt, Xopt, poses = lmpc_solver.solve(T_current, T_des)

    #Solve QP
    qp_solver.update_robot_state(panda)
    qp_solver.solve(Uopt[0:6], alpha=0.02, beta=0.01)
    panda.qd = qp_solver.solution
    
    #Simulate
    env.step(dt)
