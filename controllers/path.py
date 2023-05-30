import numpy as np
import scipy.interpolate as si
import scipy.optimize as opt
from dynamic_systems import Integrator


def Line(gamma, params):
    '''
    Method for a straight line path.
    '''
    starting_pt = params["start"]
    direction = params["direction"]
    direction = direction/np.linalg.norm(direction)

    xd = starting_pt + gamma*direction
    dxd = direction
    # ddxd = np.zeros(len(direction))

    return xd, dxd


def Circle(gamma, params):
    '''
    Method for a circular path. 
    gamma = 0 corresponds to the top point on the circle.
    '''
    center_pt = params["center"]
    radius = params["radius"]

    xd = np.array(center_pt) + radius*np.array([ np.sin(gamma), np.cos(gamma) ])
    dxd = radius*np.array([ np.cos(gamma), -np.sin(gamma) ])
    # ddxd = (1/radius)*np.array([ -np.sin(gamma/radius), np.cos(gamma/radius) ])

    return xd, dxd


def BSpline(gamma, params):
    '''
    Class for B-spline based paths.
    '''
    def getControlPoints(knots, k):
        n = len(knots) - 1 - k
        cx = np.zeros(n)
        for i in range(n):
            tsum = 0
            for j in range(1, k+1):
                tsum += knots[i+j]
            cx[i] = float(tsum)/k
        return cx

    degree = params["degree"]
    pts = np.array( params["points"] )

    knotspace = range(pts.shape[0])

    # find knot vector 
    knots = si.InterpolatedUnivariateSpline(knotspace, knotspace, k=degree).get_knots()
    knots_full = np.concatenate(([knots[0]]*degree, knots, [knots[-1]]*degree))

    # define tuples of knot vector, coefficient vector (control points coordinates) and spline degree
    tckX = knots_full, pts[:,0], degree
    tckY = knots_full, pts[:,1], degree

    # construct spline functions
    splineX = si.BSpline(*tckX)
    splineY = si.BSpline(*tckY)

    dsplineX = splineX.derivative()
    dsplineY = splineY.derivative()

    xd = np.array([ splineX(gamma), splineY(gamma) ])
    dxd = np.array([ dsplineX(gamma), dsplineY(gamma) ])

    return xd, dxd


class Path:
    '''
    This class implements path functionality.
    '''
    def __init__(self, function, params, init_path_state = [0.0]):

        # Store path function and path parameters
        self.path_func = function
        self.params = params

        # Function to compute the gradient norm
        def gradient_norm(gamma):
            _ , dxd = self.path_func(gamma, self.params)
            return np.linalg.norm( dxd )
        self.gradient_norm_func = gradient_norm

        # Parameter limits
        self.gamma_min = 0.0
        if self.path_func == Line:
            self.gamma_max = np.inf
        elif self.path_func == Circle:
            self.gamma_max = 2*np.pi
        elif self.path_func == BSpline:
            pts = np.array( params["points"] )
            self.gamma_max = pts.shape[0] - 1

        # Define parameter dynamics and initialize parameters
        self.system = Integrator(initial_state=init_path_state, initial_control = [0.0])
        self.gamma = init_path_state[0]
        self.dgamma = 0.0

        # Initialize parameter logs
        self.logs = {"gamma":[], "dgamma":[], "ddgamma":[]}

    def get_path_state(self):
        '''
        Function for getting the path states (gamma and dgamma)
        '''
        state = self.system.get_state()
        self.gamma = state[0]

        return self.gamma

    def get_path_control(self):
        '''
        Function for getting the path control (ddgamma)
        '''
        return self.dgamma

    def set_path_state(self, gamma):
        '''
        Sets (gamma, dgamma) to initial values.
        '''
        self.gamma = gamma
        self.system.set_state(self.gamma)

    def get_path_point(self, gamma):
        '''
        Returns the current virtual point position and gradient.
        '''
        return self.path_func(gamma, self.params)

    def update(self, gamma_ctrl, sample_time):
        '''
        Updates of the path.
        '''
        self.dgamma = gamma_ctrl

        self.system.set_control(gamma_ctrl) 
        self.system.actuate(sample_time)

        self.get_path_state()

        self.log_path()

    def log_path(self):
        '''
        Logs the path variables.
        '''
        self.logs["gamma"] = self.system.state_log[0]
        self.logs["dgamma"] = self.system.control_log[0]

        return self.logs
    
    def draw_path(self, path_graph, plot_params):
        '''
        Draws path using plot_params.
        '''
        numpoints = plot_params["numpoints"]
        path_length = plot_params["path_length"]

        xpath, ypath = [], []
        for k in range(numpoints):
            gamma = k*path_length/numpoints
            pos, _ = self.get_path_point(gamma)
            xpath.append(pos[0])
            ypath.append(pos[1])
        path_graph.set_data(xpath, ypath)

    def find_min(self, point, init=0.0):
        '''
        Method for computing minimum distance to point. Returns:
        minimizer (parameter solution)
        closest point in the curve
        minimum distance in path
        '''
        def cost(gamma):
            xd, = self.path_func(gamma, self.params)
            return np.norm( xd - point )
        
        results = opt.minimize( cost, init, constraints=opt.LinearConstraint( np.array(1), lb=self.gamma_min, ub=self.gamma_max ) )
        minimizer = results.x
        closest_point = self.get_path_point(minimizer)
        min_distance = cost(minimizer)

        return minimizer, closest_point, min_distance