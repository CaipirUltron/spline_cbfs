import numpy as np
from common import Rect, EllipticalBarrier, BarrierGrid
from dynamic_systems import Unicycle
from controllers import PFController, SplinePath, SplineBarrier, Supervisor
from simulations.load_splines import *

##################################### Configure and create robots and barriers #############################################
pos_offset = 1.0
robot1 = Unicycle( initial_state=[ -9.0, 10.0, 0.0 ], initial_control=np.zeros(2), geometry=Rect([3.0, 1.5], 1.0) )
robot2 = Unicycle( initial_state=[ -9.0,  5.0, 0.0 ], initial_control=np.zeros(2), geometry=Rect([3.0, 1.5], 1.0) )
robot3 = Unicycle( initial_state=[ -9.0,  0.0, 0.0 ], initial_control=np.zeros(2), geometry=Rect([3.0, 1.5], 1.0) )

robots = [ robot1, robot2, robot3 ]

connectivity = [ [1,2], [0,2], [0,1] ] # full connectivity btw 3 vehicles
# priorities = [ 1, 2, 3 ]
priorities = None

barrier1, barrier2, barrier3 = EllipticalBarrier(shape=[2.0, 1.2]), EllipticalBarrier(shape=[2.0, 1.2]), EllipticalBarrier(shape=[2.0, 1.2])
barrier_grid = BarrierGrid(barriers = [barrier1, barrier2, barrier3])

################################################### Define paths ###########################################################
paths, recommended_init_positions = [], []
print(str(len(path_params)) + " available paths: ")
for path_param in path_params:
    # print(path_params)
    path = SplinePath( params=path_param, init_path_state=[0.0] )
    paths.append(path)
    recommended_init_positions.append( path.get_path_point(0.0) )

robots[0].set_state( np.hstack([ recommended_init_positions[0], -np.pi/4 ]) )
robots[1].set_state( np.hstack([ recommended_init_positions[1], -np.pi/4 ]) )
robots[2].set_state( np.hstack([ recommended_init_positions[2], -np.pi/4 ]) )

########################################### Define spline-based barriers ###################################################
radius = 1.0

spline_barriers = []
print(str(len(barrier_params)) + " available barriers: ")
for barrier_param in barrier_params:
    print(barrier_params)
    spline_barriers.append( SplineBarrier( params=barrier_param, init_path_state=[0.0], threshold=radius) )

######################################### Configure and create controllers #################################################
sample_time = 0.01

controller_parameters1 = { 
    "sample_time": sample_time, "path": paths[0], "path_speed": 6.0,
    "q1": 1.0, "q2": 10.0, "alpha": 40.0, "beta1": 4.0, "beta2": 4.0, "kappa": 0.01, "connectivity": connectivity[0] }

controller_parameters2 = { 
    "sample_time": sample_time, "path": paths[1], "path_speed": 8.0,
    "q1": 1.0, "q2": 10.0, "alpha": 40.0, "beta1": 4.0, "beta2": 4.0, "kappa": 0.01, "connectivity": connectivity[1] }

controller_parameters3 = { 
    "sample_time": sample_time, "path": paths[2], "path_speed": 8.0,
    "q1": 1.0, "q2": 10.0, "alpha": 40.0, "beta1": 4.0, "beta2": 4.0, "kappa": 0.01, "connectivity": connectivity[2] }

super = Supervisor(paths, initial_priorities=priorities)

controller1 = PFController(robots, barrier_grid, spline_barriers, controller_parameters1, id=0, supervisor=super)
controller2 = PFController(robots, barrier_grid, spline_barriers, controller_parameters2, id=1, supervisor=super)
controller3 = PFController(robots, barrier_grid, spline_barriers, controller_parameters3, id=2, supervisor=super)

controllers = [ controller1, controller2, controller3 ]

####################################################### Configure plot #####################################################
xoffsets, yoffsets = [-5, 5], [-5, 5]
xlimits = (np.array(xlimits) + np.array(xoffsets)).tolist()
ylimits = (np.array(ylimits) + np.array(yoffsets)).tolist()
plot_config = {
    "figsize": (5,5),
    "gridspec": (1,1,1),
    "widthratios": [1],
    "heightratios": [1],
    "axeslim": tuple(xlimits+ylimits),
    "path_length": 10,
    "numpoints": 1000,
    "drawlevel": False,
    "resolution": 50,
    "fps":120,
    "pad":2.0
}