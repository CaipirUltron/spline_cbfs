import numpy as np

from common import Rect
from dynamic_systems import Unicycle
from controllers import PFController, SplinePath, SplineBarrier
from simulations.load_splines import *

########################################### Configure and create robots ####################################################
pos_offset = 1.0
robot1 = Unicycle( initial_state=[ -5.0, 10.0, 0.0 ], initial_control=np.zeros(2), geometry=Rect([3.0, 1.5], 1.0) )

robots = [ robot1 ]

############################ Loaded spline parameters: path_params, barrier_params #########################################
paths = []
print("Available paths: ")
for path_param in path_params:
    print(path_params)
    paths.append( SplinePath( params=path_param, init_path_state=[0.0] ) )

radius = 1.0

barriers = []
print("Available barriers: ")
for barrier_param in barrier_params:
    print(barrier_params)
    barriers.append( SplineBarrier( params=barrier_param, init_path_state=[0.0], threshold=radius) )


############################################# Configure and create controllers #############################################
path_speed = 5.5
sample_time = 0.005

controller_parameters1 = { 
    "sample_time": sample_time, "path": paths[0], "path_speed": path_speed,
    "q1": 1.0, "q2": 10.0, "alpha1": 100.0, "alpha2": 10.0, "beta0": 1.0, "radius": radius }

# Create QP controller and graphical simulation.
controller1 = PFController(robots, barriers, controller_parameters1, id=0)

controllers = [ controller1 ]

####################################################### Configure plot #####################################################
xoffsets, yoffsets = [-5, 5], [-5, 5]
xlimits = (np.array(xlimits) + np.array(xoffsets)).tolist()
ylimits = (np.array(ylimits) + np.array(yoffsets)).tolist()
plot_config = {
    "axeslim": tuple(xlimits+ylimits),
    "path_length": 100, 
    "numpoints": 1000,
    "radius": radius,
}