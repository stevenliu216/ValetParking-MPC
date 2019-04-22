import matplotlib.pyplot as plt
import logging

from src.parameters import TARGET_SPEED, SHOW_PLOTS
from src.simulator import simulate
from src.util import *

def main():

    logging.basicConfig(filename='mpc.log', filemode='w', level=logging.INFO)
    # Step 1 - Generate a test track
    '''
    # Circular track
    xlist = [0.0, 60.0, 60.0, 0.0, 0.0, 30.0, 31.0]
    ylist = [0.0, 0.0, 50.0, 65.0, 30.0, 30.0, 31.0]
    test_track = get_test_track(xlist, ylist) # test_track is [rx, ry, rphi, rk]
    '''

    '''
    # Switch back track
    xlist = [0.0, 30.0, 6.0, 20.0, 35.0]
    ylist = [0.0, 0.0, 20.0, 35.0, 20.0]
    test_track = get_test_track(xlist, ylist) # test_track is [rx, ry, rphi, rk]
    '''

    
    # Test building 2
    xlist = [355.47, 357.78, 367.03, 389.0, 427.15, 476.87, 513.87, 564.74, 591.33, 610.99, 622.55, 629.49, 629.49, 630.64, 629.49, 638.74, 656.08, 697.7, 732.39, 1252.68, 1357.89, 1396.05, 1431.89, 1429.58, 1422.64, 1405.3, 1367.14, 1141.68, 1080.41, 990.22, 891.94, 814.48, 758.98, 733.54, 720.83, 708.11, 697.7, 696.55, 693.08, 696.55, 711.58, 727.76, 743.95, 778.64, 1054.97, 1290.83, 1305.86, 1310.49, 1315.11, 1318.58, 1320.9, 1323.21, 1323.21, 1324.36, 1323.21, 1323.21, 1323.21, 1323.21]
    ylist = [244.59, 270.02, 300.08, 328.99, 346.33, 346.33, 342.86, 345.17, 344.02, 340.55, 324.36, 293.15, 259.62, 216.84, 192.56, 171.74, 160.18, 154.4, 153.25, 156.71, 157.87, 186.78, 223.77, 241.12, 248.05, 253.83, 259.62, 252.68, 252.68, 253.83, 250.37, 248.05, 245.74, 248.05, 257.3, 266.55, 288.52, 303.55, 330.14, 345.17, 353.27, 356.74, 357.89, 357.89, 361.36, 363.67, 362.52, 361.36, 356.74, 352.11, 347.49, 342.86, 338.24, 333.61, 327.83, 324.36, 318.58, 315.11]
    xlist = [t/30 for t in xlist]
    ylist = [-t/30 for t in ylist] # image coordinates to cartesian
    test_track = get_test_track(xlist, ylist)
    logging.info('Step 1 - Generating a test track')
    
    if SHOW_PLOTS:
        plt.figure(1, figsize=(12,8))
        plt.subplot(2,3,1)
        plt.plot(xlist, ylist, 'x')
        plt.plot(test_track[0], test_track[1])
        plt.axis('equal')

    # Step 2 - Generate a speed profile for test track
    speed = generate_speed_profile(test_track, TARGET_SPEED)
    logging.info('Step 2 - Generating target speeds for test track')

    if SHOW_PLOTS:
        plt.subplot(2,3,4)
        

    # Step 3 - Run simulation
    logging.info('Step 3 - Starting the simulation')
    simulated_result = \
        simulate(test_track, speed)

    # Step 4 - plot
    if SHOW_PLOTS:
        plt.show()
        input('Press any key to quit...')

if __name__=='__main__':
    main()