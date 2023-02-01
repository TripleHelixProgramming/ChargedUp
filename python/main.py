from single_jointed_arm import single_jointed_arm
from arm_sim import animate_arm_trajectory
from math import *
from math_util import RK4
from arm_control_law import P

time_horizon = 100.0
dt = 0.005

initialState = [0, 0]
referenceState = [1, 0]

arm = single_jointed_arm(2.0, 5.0, 0.01, 1.0)

def main():

    states = [initialState, initialState, initialState]

    for iterate in range(int(time_horizon / dt)):
        # Add 10 ms of latency
        state = states[-3]
        states.append(RK4(arm.dynamics, state, [P(state[0], referenceState[0])], dt))

    animate_arm_trajectory(int(1.0 / dt), dt, states, arm, [referenceState[0], 0], False)

main()