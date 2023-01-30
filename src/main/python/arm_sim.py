import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

def animate_arm(timestamps, states):
    for time, state in zip(timestamps, states):
        print(time)