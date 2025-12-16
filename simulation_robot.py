#!/usr/bin/env python3
import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from MPC.QP_solver import QPController
import spatialgeometry as sg

# Init env
env = swift.Swift()
env.launch(realtime=True)
panda = rtb.models.Panda()
panda.q = panda.qr
arrived = False
env.add(panda)
dt = 0.01

# Init desired position
T_ini = panda.fkine(panda.q)
Tdes = panda.fkine(panda.q)
target = sg.Sphere(radius=0.02, pose=Tdes, color=[0,1,0])
env.add(target)
env.set_camera_pose([1.0, 1.0, 0.7], [0, 0, 0.4])

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
    v, _ = rtb.p_servo(panda.fkine(panda.q), Tdes, 1, 0.01)
    
    #Solve pseudo inverse
    panda.qd = np.linalg.pinv(panda.jacobe(panda.q)) @ v
    
    #Simulate
    env.step(dt)