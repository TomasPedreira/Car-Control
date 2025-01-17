#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import qwiic_i2c
import time
import numpy as np
import controllerFuns as cf
from scipy.integrate import solve_ivp

## ---------- Python Definition of MATLAB wrapToPi() function ---------- ##
def wrapToPi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

## ---------- Car Model Equation ---------- ##
def ssCar(x, u):
    b = 0.2
    vel = 0.5 * (u[0] + u[1])
    ang_vel = (u[0] - u[1]) / b
    return np.array([
        vel * np.cos(x[2]),
        vel * np.sin(x[2]),
        ang_vel
    ])

def controller_node():
    rospy.init_node('jetbot_controller', anonymous=True)
    
    Ts = 0.005
    k = 1 # Simulation Iteration
    
    # ----- Initialize controller ----- #
    rate = rospy.Rate(Ts)
    
    # ---------- Calculate Robot Car References ---------- #
    r = 2
    v_target = 0.1
    omega = 2 * np.pi / 40
    total_time = 80
    N = int(total_time / Ts)
    
    t = np.linspace(0, total_time, N + 1)
    X_target = -5 + v_target * t
    Y_target = np.zeros_like(t)
    
    # 1st Car Reference
    X1 = X_target + r * np.cos(omega * t)
    Y1 = Y_target + r * np.sin(omega * t)
    
    # 2nd Car Reference
    X2 = X_target + r * np.cos(omega * t + np.pi)
    Y2 = Y_target + r * np.sin(omega * t + np.pi)
    
    # ---------- Controller Initialization ---------- #
    time = np.arange(Ts, 80 + Ts, Ts)
    T = len(time)
    
    # Array Size
    nu = 4
    nx = 6
    
    # Robot Cars + Target Vehicle Initial Positions
    extraCar = np.array([-5, 0, 0])
    firstCar = np.array([-3, -2, np.pi/2])
    secondCar = np.array([-7, 2, -np.pi/2])
    
    # Target Vehicle Reference
    # extraCarRef = np.vstack([-5 + 0.1 * time, np.zeros_like(time)])
    
    # Tracking Car References
    car_ref = np.vstack([X1, Y1, X2, Y2])
    ref1 = car_ref[0:2, :]
    ref2 = car_ref[2:4, :]
    
    # Simulation Variables
    utraj = np.zeros((nu, T))
    xtraj = np.zeros((nx, T+1))
    xtraj[:3, 0] = firstCar
    xtraj[3:6, 0] = secondCar
    xtraj[:3, 1] = firstCar
    xtraj[3:6, 1] = secondCar
    
    # Initialize Control Action Variables
    v1 = 0
    v2 = 0
    alpha1 = 0
    alpha2 = 0
    
    rospy.loginfo("Controller Initialized.")
    while not rospy.is_shutdown():
        # ----- Main loop ----- #
        # Data acquisition example
        sensData = cf.getJetsPosition([0, 1]) # Returns a dictionary with the position
        # of the jetbots [[jetID, x, y, yaw], [jetID, x, y, ...]]. Receives a list with
        # the jet IDs to retrieve positions for.
        sensData = np.array(sensData)
        
        conData = cf.getJetbotControlAction([0, 1]) # Returns a dictionary with the
        # control action of the jetbots [[jetID, v, w], [jetID, v, w], ...]. Receives a list
        # with the jet IDs to retrieve control actions for.
        conData = np.array(conData)
        
        sensData0 = sensData[0]
        sensData1 = sensData[1]
        conData0 = conData[0]
        conData1 = conData[1]
        
        cf.sendJetbotControlAction([0, v1, alpha1], [1, v2, alpha2]) # Sends the control action to the jetbots [[jetID, v, w], [jetID, v, w]]
        
        # ---------- USER CODE STARTS HERE ---------- #
        xtraj[0:3, k] = sensData0[1:3]
        xtraj[3:6, k] = sensData1[1:3]
        
        # 1st Car Pure Pursuit Algorithm
        error_norm1 = np.linalg.norm(ref1[:, k] - xtraj[0:2, k])
        ref_aux1 = ref1[:, k] - xtraj[0:2, k]
        error_orientation1 = wrapToPi(np.arctan2(ref_aux1[1], ref_aux1[0]) - xtraj[2, k])
        v1 = min(5, error_norm1 * 0.5)
        alpha1 = np.arctan2(2 * 0.2 * np.sin(error_orientation1), 0.4)
        v1r = v1 + alpha1 * 0.2 / 2
        v1l = v1 - alpha1 * 0.2 / 2

        # 2nd Car Pure Pursuit Algorithm
        error_norm2 = np.linalg.norm(ref2[:, k] - xtraj[3:5, k])
        ref_aux2 = ref2[:, k] - xtraj[3:5, k]
        error_orientation2 = wrapToPi(np.arctan2(ref_aux2[1], ref_aux2[0]) - xtraj[5, k])
        v2 = min(5, error_norm2 * 0.5)
        alpha2 = np.arctan2(2 * 0.2 * np.sin(error_orientation2), 0.4)
        v2r = v2 + alpha2 * 0.2 / 2
        v2l = v2 - alpha2 * 0.2 / 2

        utraj[:, k] = np.array([v1r, v1l, v2r, v2l])
        k += 1
        # ---------- USER CODE ENDS HERE ---------- #
        rate.sleep()

if __name__ == '__main__':
    try:
        controller_node()
    except rospy.ROSInterruptException:
        pass