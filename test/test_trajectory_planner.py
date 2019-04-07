from src.trajectory_planner import generate_trajectory

def test_spline():
    '''Given 2D waypoints, return a reference trajectory'''

    # This tests a straight trajectory
    xlist = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ylist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # result is a tuple of 4 lists
    result = generate_trajectory(xlist, ylist)
    assert(len(result[0])==50)
    assert(result[0] == [_ for _ in range(0, 50)])