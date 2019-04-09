from src.trajectory_planner import *
from src.simulator import simulate

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed

def main():
    xlist = [0.0, 10.0, 20.0]
    ylist = [0.0, 0.0, 0.0]
    rx, ry, rphi, rk = generate_trajectory(xlist, ylist)
    speed = generate_speed_profile(rx, ry, rphi, TARGET_SPEED)
    simulated_result = \
        simulate(rx, ry, rphi, rk, speed, dl=1.0)

if __name__=='__main__':
    main()