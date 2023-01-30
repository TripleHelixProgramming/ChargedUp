import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from single_jointed_arm import single_jointed_arm
from math import *
import os

def animate_arm_trajectory(
    fps, dt, states, arm
    ):
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.set_title("Single-DOF Arm Simulation")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(-arm.l * 1.5, arm.l * 1.5)
    ax.set_ylim(-arm.l * 1.5, arm.l * 1.5)
    ax.set_aspect('equal', adjustable='box')

    arm_line, = ax.plot([0, 0], [0, 0], 'red')

    def animate(i):
        state = states[i]
        theta = state[0]
        arm_line.set_data([0, arm.l * cos(theta)], [0, arm.l * sin(theta)])
        return [arm_line]

    
    anim = FuncAnimation(fig,
                        animate, 
                        frames = len(states),
                        interval = int(dt * 1000),
                        blit = True)

    if not os.path.exists("src/main/python/animations"):
        os.makedirs("src/main/python/animations")
    anim.save(
        os.path.join("src/main/python/animations", "{}.mp4".format("One-DOF Arm Simulation")),
        writer="ffmpeg",
        dpi=250,
        fps=fps,
    )

    plt.show()