import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants
mp = 1.0   # Pendulum mass
d = 0.5    # Distance to COM
Ip = 1.0   # Pendulum moment of inertia
Bp = 0.0   # Damping
l = 1.0    # Arm length
g = 9.81   # Gravity

# Controller gains (from MATLAB X1 vector)
K = np.array([261.5626, -31.6228, 131.0769, -40.2996])

# Acceleration control function
def acceleration(y):
    return -np.dot(K, y)

# ODE function
def odefun(t, y):
    theta, phi, theta_dot, phi_dot = y
    A = Ip + mp * d**2
    u = acceleration(y)
    
    dydt = np.zeros(4)
    dydt[0] = theta_dot
    dydt[1] = phi_dot
    dydt[2] = (mp * l * d * np.cos(theta) * u +
               mp * d**2 * np.sin(theta) * np.cos(theta) * phi_dot**2 +
               mp * d * g * np.sin(theta) - Bp * theta_dot) / A
    dydt[3] = u
    return dydt

# Initial conditions
y0 = [np.pi/8, 0, 0, 0]
t_span = (0, 20)
t_eval = np.linspace(t_span[0], t_span[1], 400)

# Solve ODE
sol = solve_ivp(odefun, t_span, y0, t_eval=t_eval, method='RK45')

theta = sol.y[0]
phi = sol.y[1]
t = sol.t

# Compute control input over time
u = np.array([-np.dot(K, sol.y[:, i]) for i in range(sol.y.shape[1])])

# Plot angular positions
plt.figure(figsize=(10, 5))
plt.subplot(2,1,1)
plt.plot(t, theta, 'r', label='θ (pendulum)')
plt.plot(t, phi, 'b', label='ϕ (arm)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Angular Positions')
plt.legend()
plt.grid()

# Plot control input
plt.subplot(2,1,2)
plt.plot(t, u, 'k--', label='Control Input')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration')
plt.title('Control Input Over Time')
plt.grid()
plt.tight_layout()
plt.show()

# ================== 3D ANIMATION ==================

# Arm and pendulum lengths
L1 = 1.0  # Arm
L2 = 0.5  # Pendulum

# Compute arm and pendulum positions
Ax = L1 * np.cos(phi)
Ay = L1 * np.sin(phi)
Az = np.zeros_like(Ax)

Bx = Ax - L2 * np.sin(theta) * np.sin(phi)
By = Ay + L2 * np.sin(theta) * np.cos(phi)
Bz = -(-Az + L2 * np.cos(theta))

# Set up 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-L1 - L2, L1 + L2])
ax.set_ylim([-L1 - L2, L1 + L2])
ax.set_zlim([-L2, L1 + L2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Motion of the System')

line1, = ax.plot([], [], [], 'bo-', lw=2)  # Base to Arm
line2, = ax.plot([], [], [], 'ro-', lw=2)  # Arm to Pendulum

def update(i):
    line1.set_data([0, Ax[i]], [0, Ay[i]])
    line1.set_3d_properties([0, Az[i]])
    
    line2.set_data([Ax[i], Bx[i]], [Ay[i], By[i]])
    line2.set_3d_properties([Az[i], Bz[i]])
    return line1, line2

ani = FuncAnimation(fig, update, frames=len(t), interval=50)
plt.show()