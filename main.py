import matplotlib.pyplot as plt
import logging
import math

from src.parameters import TARGET_SPEED, SHOW_PLOTS
from src.simulator import simulate
from src.util import *

def main():

    logging.basicConfig(filename='mpc.log', filemode='w', level=logging.INFO)
    # Step 1 - Generate a test track
    '''
    # Test Ford campus start to goal - About 9 pixels is 1 meter on this image
    xlist = [139.31, 132.25, 188.74, 305.26, 397.07, 510.06, 573.62, 580.68, 580.68, 607.16, 668.95, 781.94, 898.46, 1094.43, 1322.17, 1594.06, 1795.32, 1924.2, 1936.56, 1955.98, 1961.27, 1936.56, 1890.65, 1809.44, 1761.78, 1648.79, 1611.71, 1375.14, 1018.51, 744.87, 677.78, 677.78, 704.26, 787.24, 1018.51, 1309.82, 1645.25, 1687.63, 1719.4, 1730.0, 1737.06, 1737.06]
    ylist = [329.93, 444.68, 522.36, 541.78, 540.02, 543.55, 510.0, 314.04, 243.42, 206.34, 202.81, 209.87, 216.94, 229.29, 224.0, 218.7, 213.4, 273.43, 287.55, 328.16, 356.41, 374.06, 379.36, 372.3, 356.41, 345.81, 345.81, 352.88, 361.7, 365.23, 400.54, 504.71, 568.26, 555.9, 547.08, 541.78, 543.55, 543.55, 541.78, 531.19, 510.0, 490.58]
    xlist = [t/18 for t in xlist]
    ylist = [-t/18 for t in ylist] # image coordinates to cartesian
    test_track = get_test_track(xlist, ylist)
    '''
    
    # Test forward parking
    # For this image, about 30 pixels = 1 meter
    xlist = [1825.42, 1602.18, 1180.32, 975.14, 858.6, 715.79, 628.79, 563.14, 540.16, 527.02, 528.67, 530.31, 528.67]
    ylist = [269.16, 274.08, 280.65, 305.27, 343.02, 356.15, 356.15, 351.23, 321.68, 287.21, 259.31, 221.55, 183.8]
    xlist = [t/30 for t in xlist]
    ylist = [-t/30 for t in ylist]
    test_track = get_test_track(xlist, ylist)
    
    '''
    #forward part
    xlistf = [1815.52, 1422.34, 1166.84, 948.24, 843.21, 695.58, 634.55, 593.39, 559.32, 536.61, 529.51, 528.09]
    ylistf = [272.71, 279.81, 275.55, 267.04, 255.68, 255.68, 265.62, 276.97, 296.84, 330.91, 382.01, 386.27]
    xlistf = [t/30 for t in xlistf]
    ylistf = [-t/30 for t in ylistf]
    #reverse part
    xlistr = [528.09, 521.01, 521.01, 521.01]
    ylistr = [386.27, 306.94, 214.55, 122.16]
    xlistr = [t/30 for t in xlistr]
    ylistr = [-t/30 for t in ylistr]

    test_track = get_park_test_track(xlistf, ylistf, xlistr, ylistr)
    xlist = xlistf + xlistr
    ylist = ylistf + ylistr
    '''
    logging.info('Step 1 - Generating a test track')
    
    if SHOW_PLOTS:
        plt.figure(1, figsize=(12,8))
        plt.subplot(2,2,1)
        plt.axis('equal')

    # Step 2 - Generate a speed profile for test track
    speed = generate_speed_profile(test_track, TARGET_SPEED)
    logging.info('Step 2 - Generating target speeds for test track')

    # Step 3 - Run simulation
    logging.info('Step 3 - Starting the simulation')
    simulated_result = \
        simulate(test_track, speed)
    
    # Step 4 - Calculate Errors
    logging.info('Step 4 - Calculate Errors')
    # t, x, y, phi, v, a, delta, position_error, phi_error
    avg_position_error = sum(simulated_result[7])/len(simulated_result[7])
    avg_rotation_error = sum(simulated_result[8])/len(simulated_result[8])
    logging.info('Average Position Error: \n'.format(avg_position_error))
    logging.info('Average Rotation Error: \n'.format(avg_rotation_error))

    # Step 5 - plot
    if SHOW_PLOTS:
        plt.show()
        input('Press any key to quit...')

if __name__=='__main__':
    main()