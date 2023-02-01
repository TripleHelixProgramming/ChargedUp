import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from single_jointed_arm import single_jointed_arm
from math import *
import os

def animate_arm_trajectory(fps, dt, states, arm, reference, save):
    fig = plt.figure(figsize=[10, 5])
    axs = fig.subplots(1, 2)
    ax = axs[0]
    ax2 = axs[1]
    ax.set_title("Single-DOF Arm Simulation")
    ax2.set_title("Error")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax2.set_xlabel("t (seconds)")
    ax2.set_ylabel("error (rad)")
    ax.set_xlim(-arm.l * 1.5, arm.l * 1.5)
    ax.set_ylim(-arm.l * 1.5, arm.l * 1.5)
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    ax2.set_aspect('equal', adjustable='box')


    arm_line, = ax.plot([0, 0], [0, 0], 'red')
    ref_line, = ax.plot([0, 0], [0, 0], 'blue')

    def animate(i):
        arm_line.set_data([0, arm.l * cos(states[i][0])], [0, arm.l * sin(states[i][0])])
        ref_line.set_data([0, arm.l * cos(reference[0])], [0, arm.l * sin(reference[0])])
        return [arm_line]

    
    anim = FuncAnimation(fig,
                        animate, 
                        frames = len(states),
                        interval = int(dt * 1000),
                        blit = True)

    if save:
        if not os.path.exists("src/main/python/animations"):
            os.makedirs("src/main/python/animations")
        anim.save(
            os.path.join("src/main/python/animations", "{}.mp4".format("One-DOF Arm Simulation")),
            writer="ffmpeg",
            dpi=250,
            fps=fps,
        )

    plt.show()