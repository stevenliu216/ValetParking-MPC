import matplotlib.pyplot as plt

from src.parameters import TARGET_SPEED, SHOW_PLOTS
from src.simulator import simulate
from src.util import *

def main():
    # Step 1 - Generate a test track
    xlist = [0.0, 60.0, 60.0, 0.0, 0.0, 30.0, 31.0]
    ylist = [0.0, 0.0, 50.0, 65.0, 30.0, 30.0, 31.0]
    test_track = get_test_track(xlist, ylist) # test_track is [rx, ry, rphi, rk]
    
    if SHOW_PLOTS:
        plt.subplot(2,1,1)
        plt.plot(xlist, ylist, 'x')
        plt.plot(test_track[0], test_track[1])
        plt.title('Test Track')

    # Step 2 - Generate a speed profile for test track
    speed = generate_speed_profile(test_track, TARGET_SPEED)
    if SHOW_PLOTS:
        plt.subplot(2,1,2)
        plt.plot(speed)
        plt.title('Speed Profile')
        

    # Step 3 - Run simulation
    simulated_result = \
        simulate(test_track, speed, dl=1.0)

    # Step 4 - plot
    if SHOW_PLOTS:
        plt.show()

if __name__=='__main__':
    main()