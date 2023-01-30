from single_jointed_arm import single_jointed_arm
from arm_sim import animate_arm_trajectory
from math import *

fps = 60
dt = 1.0 / fps

initialState = [0, 0]

arm = single_jointed_arm(2, 50, 1, 1)

def main():

    states = [initialState]

    for iterate in range(fps):
        states.append([states[-1][0] + 2 * pi / fps, 0])

    animate_arm_trajectory(fps, dt, states, arm)

main()