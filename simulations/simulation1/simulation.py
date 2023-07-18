import numpy as np

from common import Rect, EllipticalBarrier
from dynamic_systems import Unicycle
from controllers import PFController, SplinePath, SplineBarrier
from simulations.load_splines import *

########################################### Configure and create robots ####################################################
pos_offset = 1.0
robot1 = Unicycle( initial_state=[ -9.0, 13.0, -np.pi/4 ], initial_control=np.zeros(2), geometry=Rect([3.0, 1.5], 1.5), barrier=EllipticalBarrier(shape=[2.0, 1.2]) )
robot2 = Unicycle( initial_state=[ -3.0, 10.0, -np.pi/4 ], initial_control=np.zeros(2), geometry=Rect([3.0, 1.5], 1.5), barrier=EllipticalBarrier(shape=[2.0, 1.2]) )

robots = [ robot1, robot2 ]

connectivity = [ [1], [0] ] # full connectivity btw 2 vehicles

############################ Loaded spline parameters: path_params, barrier_params #########################################
paths = []
print(str(len(path_params)) + " available paths: ")
for path_param in path_params:
    print(path_params)
    paths.append( SplinePath( params=path_param, init_path_state=[0.0] ) )

radius = 1.0

barriers = []
# print(str(len(barrier_params)) + " available barriers: ")
# for barrier_param in barrier_params:
#     print(barrier_params)
#     barriers.append( SplineBarrier( params=barrier_param, init_path_state=[0.0], threshold=radius) )

############################################# Configure and create controllers #############################################
sample_time = 0.005

controller_parameters1 = { 
    "sample_time": sample_time, "path": paths[0], "path_speed": 8.0,
    "q1": 1.0, "q2": 10.0, "alpha": 50.0, "beta": 10.0, "kappa": 0.1, "connectivity": connectivity[0] }

controller_parameters2 = { 
    "sample_time": sample_time, "path": paths[1], "path_speed": 10.0,
    "q1": 1.0, "q2": 10.0, "alpha": 50.0, "beta": 10.0, "kappa": 0.1, "connectivity": connectivity[1] }

# Create QP controller and graphical simulation.
controller1 = PFController(robots, barriers, controller_parameters1, id=0)
controller2 = PFController(robots, barriers, controller_parameters2, id=1)

controllers = [ controller1, controller2 ]

####################################################### Configure plot #####################################################
xoffsets, yoffsets = [-5, 5], [-5, 5]
xlimits = (np.array(xlimits) + np.array(xoffsets)).tolist()
ylimits = (np.array(ylimits) + np.array(yoffsets)).tolist()
plot_config = {
    "axeslim": tuple(xlimits+ylimits),
    "path_length": 100, 
    "numpoints": 1000,
    "fps": 120
}