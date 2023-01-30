from single_jointed_arm import single_jointed_arm

arm = single_jointed_arm(1)
dt = 0.005

def main():
    timestamps = [0]

    for iterate in range(1000):
        timestamps.append[timestamps[-1] + dt]

    print(arm.length)

main()