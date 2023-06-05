import numpy as np

from common import Rect
from dynamic_systems import Unicycle
from controllers import PFController, SplinePath

########################################### Configure and create robots ####################################################
pos_offset = 1.0
robot1 = Unicycle( initial_state=[ -5.0, 8.0, 0.0 ], initial_control=np.zeros(2), geometric_params=Rect([3.0, 1.5], 1.0) )

robots = [ robot1 ]
############################################################################################################################

################################################### Define paths ###########################################################
sample_time = 0.005

pts_x = [6, -2, 4, 6, 8, 14, 6]
pts_y = [-3, 2, 5, 0, 5, 2, -3]

# num_ctrl_pts = 10
# pts_x = np.random.randint( -10, 10, num_ctrl_pts )
# pts_y = np.random.randint( -10, 10, num_ctrl_pts )

spline_params = { "degree": 3, "points": np.array([pts_x, pts_y]).T, "orientation": "left" }
spline_path = SplinePath( params=spline_params, init_path_state=[0.0] )

paths = [ spline_path ]
############################################################################################################################

############################################# Configure and create controllers #############################################
path_speed = 1.5
radius = 1.0

controller_parameters = { 
    "sample_time": sample_time, "path": spline_path, "path_speed": path_speed,
    "q1": 1.0, "q2": 10.0, "alpha1": 100.0, "alpha2": 10.0, "beta0": 1.0, "radius": radius }

# Create QP controller and graphical simulation.
controller1 = PFController(robots, controller_parameters, id=0)

controllers = [ controller1 ]
####################################################### Configure plot #####################################################
plot_config = {
    "axeslim": (-12,12,-10,10),
    "path_length": 100, 
    "numpoints": 500,
    "radius": radius,
}