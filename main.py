from src.simulator import simulate
from src.util import *

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed

def main():
    # Step 1 - Generate a test track
    xlist = [0.0, 10.0, 20.0]
    ylist = [0.0, 0.0, 0.0]
    test_track = get_test_track(xlist, ylist) # test_track is [rx, ry, rphi, rk]
    
    # Step 2 - Generate a speed profile for test track
    speed = generate_speed_profile(test_track, TARGET_SPEED)

    # Step 3 - Run simulation
    simulated_result = \
        simulate(test_track, speed, dl=1.0)

if __name__=='__main__':
    main()