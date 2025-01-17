import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

## ---------- Python Definition of MATHLAB wrapToPi() function ---------- ##
def wrapToPi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

## ---------- Car Model Equations ---------- ##
def ssCar(x, u):
    b = 0.2
    vel = 0.5 * (u[0] + u[1])
    ang_vel = (u[0] - u[1]) / b
    return np.array([
        vel * np.cos(x[2]),
        vel * np.sin(x[2]),
        ang_vel
    ])

class Controller:
    def __init__(self):
        self.k = 0
        car_ref = carReferences()
        self.ref1 = car_ref[0:2, :]
        self.ref2 = car_ref[2:4, :]
    
    def __call__(self, xin):
        # 1st Car
        error_norm1 = np.linalg.norm(self.ref1[:, self.k] - xin[0:2])
        ref_aux1 = self.ref1[:, self.k] - xin[0:2]
        error_orientation1 = wrapToPi(np.arctan2(ref_aux1[1], ref_aux1[0]) - xin[2])
        v1 = min(5, error_norm1 * 0.5)
        alpha1 = np.arctan2(2 * 0.2 * np.sin(error_orientation1), 0.4)
        v1r = v1 + alpha1 * 0.2 / 2
        v1l = v1 - alpha1 * 0.2 / 2

        # 2nd Car
        error_norm2 = np.linalg.norm(self.ref2[:, self.k] - xin[3:5])
        ref_aux2 = self.ref2[:, self.k] - xin[3:5]
        error_orientation2 = wrapToPi(np.arctan2(ref_aux2[1], ref_aux2[0]) - xin[5])
        v2 = min(5, error_norm2 * 0.5)
        alpha2 = np.arctan2(2 * 0.2 * np.sin(error_orientation2), 0.4)
        v2r = v2 + alpha2 * 0.2 / 2
        v2l = v2 - alpha2 * 0.2 / 2

        self.k += 1
        return np.array([v1r, v1l, v2r, v2l])

## ---------- Generate Car References ---------- ##
def carReferences():
    r = 2
    v_target = 0.1
    omega = 2 * np.pi / 40
    Ts = 0.005
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
    
    return np.vstack([X1, Y1, X2, Y2])

def main():
    Ts = 0.005
    time = np.arange(Ts, 80 + Ts, Ts)
    T = len(time)
    
    # Array Size
    nu = 4
    nx = 6
    
    # Cars + Target Vehicle Initial Positions
    extraCar = np.array([-5, 0, 0])
    firstCar = np.array([-3, -2, np.pi/2])
    secondCar = np.array([-7, 2, -np.pi/2])
    
    # Target Vehicle Reference
    extraCarRef = np.vstack([-5 + 0.1 * time, np.zeros_like(time)])
    
    # Tracking Car References
    car_ref = carReferences()
    ref1 = car_ref[0:2, :]
    ref2 = car_ref[2:4, :]
    
    # Simulation Variables
    utraj = np.zeros((nu, T))
    xtraj = np.zeros((nx, T+1))
    xtraj[:3, 0] = firstCar
    xtraj[3:6, 0] = secondCar
    xtraj[:3, 1] = firstCar
    xtraj[3:6, 1] = secondCar
    
    controller = Controller()
    
    # Simulation Loop
    for k in range(1, T):
        utraj[:, k] = controller(xtraj[:, k])
        
        # ODE45 Integration for 1st Car
        sol = solve_ivp(
            fun=lambda t, y: ssCar(y, utraj[0:2, k]),
            t_span=[0, Ts],
            y0=xtraj[0:3, k],
            method='RK45'
        )
        xtraj[0:3, k+1] = sol.y[:, -1]
        
        # ODE45 Integration for 2nd Car
        sol = solve_ivp(
            fun=lambda t, y: ssCar(y, utraj[2:4, k]),
            t_span=[0, Ts],
            y0=xtraj[3:6, k],
            method='RK45'
        )
        xtraj[3:6, k+1] = sol.y[:, -1]
    
    ## ------------ Plot Results ------------ ##
    plt.figure(figsize=(10, 8))
    
    # Cars References
    plt.plot(ref1[0, :], ref1[1, :], 'r--', linewidth=1.4, label='Reference Car 1')
    plt.plot(ref2[0, :], ref2[1, :], 'b--', linewidth=1.4, label='Reference Car 2')
    
    # Cars + Target Vehicle References
    plt.plot(xtraj[0, :], xtraj[1, :], 'r-', alpha=0.5, linewidth=2, label='Car 1')
    plt.plot(xtraj[3, :], xtraj[4, :], 'b-', alpha=0.5, linewidth=2, label='Car 2')
    plt.plot(extraCarRef[0, :], extraCarRef[1, :], 'm-', alpha=0.5, linewidth=2, label='Target Vehicle Car')
    
    # Cars + Target Vehicle Final Positions
    plt.plot(xtraj[0, -1], xtraj[1, -1], 'ro', markersize=6, label='Car 1 End')
    plt.plot(xtraj[3, -1], xtraj[4, -1], 'bo', markersize=6, label='Car 2 End')
    plt.plot(extraCarRef[0, -1], extraCarRef[1, -1], 'mo', markersize=6, label='Target Vehicle End')
    
    plt.grid(True)
    plt.xlabel('X Axis (m)')
    plt.ylabel('Y Axis (m)')
    plt.title('Robot Car Trajectory')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()