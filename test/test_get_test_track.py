from src.util import get_test_track

import math

def test_spline_straight():
    '''Given 2D waypoints, return a reference trajectory'''

    # This tests a straight trajectory
    xlist = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ylist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # result is a tuple of 4 lists
    result = get_test_track(xlist, ylist)
    assert(len(result[0])==50)
    assert(result[0] == [_ for _ in range(0, 50)])

def test_spline_parking():
    '''Picked waypoints that resemble a parking lot'''
    xlist = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    ylist = [0.0, 0.0, 0.5, 1.0, 3.0, 5.0, 8.0]
    result = get_test_track(xlist, ylist)

    xlist2 = [6.0, 5.2, 5.0, 5.0, 5.0, 5.0]
    ylist2 = [8.0, 3.0, 0.0, -1.0, -2.0, -3.0]
    result2 = get_test_track(xlist2, ylist2)

    result[0].extend(result2[0])
    result[1].extend(result2[1])
    result[2].extend(result2[2])
    result[3].extend(result2[3])

    for xx in result[0]: 
        res = math.isnan(xx)
    assert(res != True)
    for yy in result[1]: 
        res = math.isnan(yy)
    assert(res != True)
    for pp in result[2]: 
        res = math.isnan(pp)
    assert(res != True)
    for kk in result[3]: 
        res = math.isnan(kk)
    assert(res != True)