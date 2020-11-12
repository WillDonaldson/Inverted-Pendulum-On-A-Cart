import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
from scipy.integrate import solve_ivp
from time import time

import variables

fig = plt.figure()
ax = fig.add_subplot(111, aspect = 'equal', xlim = (-15, 15), ylim = (-6, 6), title = "Inverted Pendulum Simulation")
ax.grid()

# animation parameters
origin = [0.0, 0.0]
dt = 0.05
frames = 200
t_span = [0.0, frames * dt]

pendulumArm = lines.Line2D(origin, origin, color='r') 
cart = patches.Rectangle(origin, 1.5, 0.5, color='b')
def stateSpace(t, initial):
    # returns state space model evaluated at initial conditions
    ss = np.matmul(variables.A, np.transpose(initial))
    return ss

initial=np.array([0, 7*np.pi/8, 0, 0])
ts = np.linspace(t_span[0], t_span[1], frames)
pendulum_state = solve_ivp(stateSpace, t_span, initial, t_eval = ts, rtol=1e-8)

def init():
    ax.add_patch(cart)
    ax.add_line(pendulumArm) 
    return pendulumArm, cart

def animate(i):
    xPos = pendulum_state.y[0][i] 
    theta = pendulum_state.y[1][i]
    x = [origin[0] + xPos, origin[0] + xPos + variables.L * np.sin(theta)]
    y = [origin[1], origin[1] - variables.L * np.cos(theta)]
    pendulumArm.set_xdata(x)
    pendulumArm.set_ydata(y)
    cartPos = [origin[0] + xPos - cart.get_width()/2, origin[1] - cart.get_height()]
    cart.set_xy(cartPos)
    return pendulumArm, cart

t0 = time()
animate(0)                          #sample time required to evaluate function
t1 = time()
interval = 1000 * dt - (t1 - t0)

anim = animation.FuncAnimation(fig, animate, init_func = init, frames = frames, interval = interval, blit = True)
plt.show()