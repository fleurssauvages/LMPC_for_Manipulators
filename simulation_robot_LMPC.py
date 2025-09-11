import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from MPC.QP_solver import QPController
import spatialgeometry as sg
from MPC.LMPC_solver import LinearMPCController

# Init env
env = swift.Swift()
env.launch(realtime=True)
panda = rtb.models.Panda()
panda.q = panda.qr
arrived = False
env.add(panda)
dt = 0.05

# Init desired position
T_ini = panda.fkine(panda.q)
Tdes = panda.fkine(panda.q)
target = sg.Sphere(radius=0.02, pose=Tdes, color=[0,1,0])
env.add(target)
env.set_camera_pose([1.0, 1.0, 0.7], [0, 0, 0.4])

# Init LMPC solver for path planning, gamma the gain of the controller (like K in proportionnal), the lower the faster
lmpc_solver = LinearMPCController(horizon=20, dt=dt, gamma=0.1,
                                    u_min=np.array([-0.5, -0.5, -0.5, -1.0, -1.0, -1.0]),
                                    u_max=np.array([ 0.8,  0.8,  0.8,  1.0,  1.0,  1.0]))
Uopt, Xopt, poses = lmpc_solver.solve(np.array(T_ini), np.array(Tdes))

# Init QP solver for IK with safety
qp_solver = QPController(panda)
qp_solver.solve(np.zeros((6,1)))

# Add sliders to control desired position
x, y, z = 0.0, 0.0, 0.0
def set_x(x_set):
    global Tdes, target, x, y, z
    x, Tdes = float(x_set), T_ini * sm.SE3.Trans(float(x_set), y, z)
    target.T = Tdes
env.add(swift.Slider(lambda x: set_x(x),min=-0.4,max=0.4,step=0.01,desc="x",))
def set_y(y_set):
    global Tdes, target, x, y, z
    y, Tdes = float(y_set), T_ini * sm.SE3.Trans(x, float(y_set), z)
    target.T = Tdes
env.add(swift.Slider(lambda x: set_y(x),min=-0.4,max=0.4,step=0.01,desc="y",))
def set_z(z_set):
    global Tdes, target, x, y, z
    z, Tdes = float(z_set), T_ini * sm.SE3.Trans(x, y, float(z_set))
    target.T = Tdes
env.add(swift.Slider(lambda x: set_z(-x),min=-0.5,max=0.5,step=0.01,desc="z",))

# Loop
while True:
    #Compute desired velocity from simple prop controller
    T = panda.fkine(panda.q)
    Uopt, Xopt, poses = lmpc_solver.solve(T, Tdes)
    
    #Solve QP
    qp_solver.update_robot_state(panda)
    qp_solver.solve(Uopt[0:6], alpha=0.02, beta=0.01)
    panda.qd = qp_solver.solution
    
    #Simulate
    env.step(dt)