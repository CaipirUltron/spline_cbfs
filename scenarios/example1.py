import numpy as np

from geometry import Rect
from dynamic_systems import Unicycle
from controllers import PFController, Path, Circle

########################################### Configure and create robots ####################################################
pos_offset = 1.0
robot1 = Unicycle( initial_state=[ -9.0, 10.0, 0.0 ], initial_control=np.zeros(2), geometric_params=Rect([3.0, 1.5], 1.0) )
robot2 = Unicycle( initial_state=[ -9.0,  5.0, 0.0 ], initial_control=np.zeros(2), geometric_params=Rect([3.0, 1.5], 1.0) )
robot3 = Unicycle( initial_state=[ -9.0,  0.0, 0.0 ], initial_control=np.zeros(2), geometric_params=Rect([3.0, 1.5], 1.0) )

robots = [ robot1, robot2, robot3 ]
############################################################################################################################

################################################### Define paths ###########################################################
sample_time = 0.005

circle1_params = { "center": [0.0, 0.0], "radius": 10.0 }
circle2_params = { "center": [0.0, 0.0], "radius": 6.0 }
circle3_params = { "center": [0.0, 0.0], "radius": 3.0 }

path1 = Path( function=Circle, params=circle1_params, init_path_state=[0.0] )
path2 = Path( function=Circle, params=circle2_params, init_path_state=[0.0] )
path3 = Path( function=Circle, params=circle3_params, init_path_state=[0.0] )

paths = [ path1, path2, path3 ]
############################################################################################################################

############################################# Configure and create controllers #############################################
path_speed = 2.0
radius = 0.1

controller_parameters1 = { 
    "sample_time": sample_time, "path": path1, "path_speed": path_speed,
    "q1": 1.0, "q2": 10.0, "alpha1": 100.0, "alpha2": 10.0, "beta0": 1.0, "radius": radius }

controller_parameters2 = { 
    "sample_time": sample_time, "path": path2, "path_speed": path_speed,
    "q1": 1.0, "q2": 10.0, "alpha1": 100.0, "alpha2": 10.0, "beta0": 1.0, "radius": radius }

controller_parameters3 = { 
    "sample_time": sample_time, "path": path3, "path_speed": path_speed,
    "q1": 1.0, "q2": 10.0, "alpha1": 100.0, "alpha2": 10.0, "beta0": 1.0, "radius": radius }

# Create QP controller and graphical simulation.
controller1 = PFController(robots, controller_parameters1, id=0)
controller2 = PFController(robots, controller_parameters2, id=1)
controller3 = PFController(robots, controller_parameters3, id=2)

controllers = [ controller1, controller2, controller3 ]
####################################################### Configure plot #####################################################
plot_config = {
    "axeslim": (0,12,-6,6),
    "path_length": 100, 
    "numpoints": 500,
    "radius": radius,
}