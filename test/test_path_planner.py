import numpy as np

from src.util import vehicle_state, get_test_track, generate_speed_profile
from src.path_planner import PathPlanner
from src.parameters import T

def test_calc_ref_trajectory_1():
    state = vehicle_state(0.0, 0.0, 0.0, 0.0)
    xlist = [0.0, 1.0, 2.0, 3.0]
    ylist = [0.0, 0.0, 0.0, 0.0]
    test_track = get_test_track(xlist, ylist)
    speed = generate_speed_profile(test_track, 10/3.6)
    path_planner = PathPlanner(state, test_track, speed, 0)
    result = path_planner.calc_ref_trajectory()

    assert(np.shape(result) == (4,T+1))


def test_calc_ref_trajectory_2():
    state = vehicle_state(0.1, 0.0, 0.0, 0.0)
    xlist = [0.0, 1.0, 2.0, 3.0, 4.0]
    ylist = [0.0, 0.0, 0.0, 0.0, 0.0]
    test_track = get_test_track(xlist, ylist)
    speed = generate_speed_profile(test_track, 10/3.6)
    path_planner = PathPlanner(state, test_track, speed, 0)
    result = path_planner.calc_ref_trajectory()

    assert(np.shape(result) == (4,T+1))