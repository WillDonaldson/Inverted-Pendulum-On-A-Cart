import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
from time import time

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

def init():
    ax.add_patch(cart)
    ax.add_line(pendulumArm) 
    return pendulumArm, cart

def animate(i):
    pendulumArm.set_xdata((i, i))
    pendulumArm.set_ydata((0, -3))
    cartPos = [i - cart.get_width()/2, 0 - cart.get_height()/2]
    cart.set_xy(cartPos)
    return pendulumArm, cart


t0 = time()
animate(0)                          #sample time required to evaluate function
t1 = time()
interval = 1000 * dt - (t1 - t0)

anim = animation.FuncAnimation(fig, animate, init_func = init, frames = frames, interval = interval, blit = True)
plt.show()