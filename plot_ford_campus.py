import sys
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.image as mpimg

import src.util as u

img = mpimg.imread('ford_campus.png')

def run(n):
    '''Click waypoints and get list of x and y coordinates'''
    def onclick(event):
        global ix, iy
        ix, iy = event.xdata, event.ydata
        print("The current point is: ")
        print(ix, iy)
        coords.append((ix, iy))
        xlist.append(ix)
        ylist.append(iy)
        if event.button==3 or len(coords) == n:
            fig.canvas.mpl_disconnect(cid)
            plt.close()
        return coords
    coords = []
    xlist, ylist = [], []
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(img)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    ax.plot(xlist, ylist, 'bo')
    plt.show()
    rounded_x_list = [round(_, 2) for _ in xlist]
    rounded_y_list = [round(_, 2) for _ in ylist]
    print(rounded_x_list)
    print(rounded_y_list)

    '''Plot the trajectory on top of image'''
    test_track = u.get_test_track(rounded_x_list, rounded_y_list)
    fig2 = plt.figure(figsize=(13,7))
    plt.imshow(img)
    plt.plot(rounded_x_list, rounded_y_list, 'ro')
    plt.plot(test_track[0], test_track[1], '--b')
    plt.show()

if __name__=='__main__':
    n = int(sys.argv[1])
    run(n)
