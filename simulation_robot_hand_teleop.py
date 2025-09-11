import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import mediapipe as mp
import cv2
from MPC.QP_solver import QPController
from MPC.LMPC_solver import LinearMPCController
import spatialgeometry as sg


# Robotic Simulation Parameters
env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
panda.q = panda.qr
Tinit = panda.fkine(panda.q)
target = sg.Sphere(radius=0.02, pose=Tinit, color=[0,1,0])
env.add(target)

env.add(panda)
env.set_camera_pose([1.5, 0.0, 1.0], [0, 0, 0])
dt = 0.05

#QP solver init
qp_solver = QPController(panda)
qp_solver.solve(np.zeros((6,1)))

# Init LMPC solver for path planning, gamma the gain of the controller (like K in proportionnal), the lower the faster
lmpc_solver = LinearMPCController(horizon=25, dt=dt, gamma=0.001,
                                    u_min=np.array([-0.5, -0.5, -0.5, -1.0, -1.0, -1.0]),
                                    u_max=np.array([ 0.8,  0.8,  0.8,  1.0,  1.0,  1.0]))

# Mediapipe parameters
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
cap = cv2.VideoCapture(0)

# Loop for hand tracking and sending to robot
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
    
    # Hand Tracking Extraction
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
                
        # Flip the image horizontally for a selfie-view display.
        cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
        # If 'ESC' is pressed, exit the loop
        if cv2.waitKey(5) & 0xFF == 27:
            break
        
        #Transform hand into a robot command
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Get the coordinates of the index finger tip (landmark 8)
                index_finger_tip = hand_landmarks.landmark[8]
                
                # Map the coordinates to robot workspace
                x = (index_finger_tip.x - 0.5)*1.2
                y = (index_finger_tip.y - 0.5)*1.2
                
                Tdes = Tinit * sm.SE3.Trans(0, x, y)
                target.T = Tdes
                
                #Compute desired velocity from simple prop controller
                T = panda.fkine(panda.q)
                Uopt, Xopt, poses = lmpc_solver.solve(np.array(T), np.array(Tdes))
                
                #Solve QP
                qp_solver.update_robot_state(panda)
                qp_solver.solve(Uopt[0:6], alpha=0.02, beta=0.01)
                panda.qd = qp_solver.solution
                
                #Update Simulation
                env.step(dt)

cap.release()
env.close()