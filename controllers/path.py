import numpy as np
from common import rot
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


class SplinePath:
    '''
    Class for a B-Spline-based path.
    '''
    def __init__(self, params, init_path_state = [0.0]):

        # Store path function and path parameters
        self.path_func = BSpline
        self.params = params

        self.degree = params["degree"]
        self.orientation = params["orientation"]

        perp_rot = np.array([ [0, -1], [1, 0]])
        if self.orientation.lower() == 'left':
            self.perp_rot = perp_rot
        elif self.orientation.lower() == 'right':
            self.perp_rot = -perp_rot

        self.gamma_min = 0.0
        self.set_points( np.array( params["points"] ) )

        # Define parameter dynamics and initialize parameters
        self.system = Integrator(initial_state=init_path_state, initial_control = [0.0])
        self.gamma = init_path_state[0]
        self.dgamma = 0.0

        # Initialize parameter logs
        self.logs = {"gamma":[], "dgamma":[], "ddgamma":[]}

    def is_closed(self):
        '''
        Checks if path is closed or open.
        '''
        xd_min = self.get_path_point(self.gamma_min)
        xd_max = self.get_path_point(self.gamma_max)
        if xd_max == xd_min:
            return True
        return False

    def generate_spline(self):
        '''
        Defines the spline.
        '''
        knotspace = range(self.points.shape[0])

        # find knot vector 
        knots = si.InterpolatedUnivariateSpline(knotspace, knotspace, k=self.degree).get_knots()
        knots_full = np.concatenate(([knots[0]]*self.degree, knots, [knots[-1]]*self.degree))

        # define tuples of knot vector, coefficient vector (control points coordinates) and spline degree
        tckX = knots_full, self.points[:,0], self.degree
        tckY = knots_full, self.points[:,1], self.degree

        # construct spline functions
        self.splineX = si.BSpline(*tckX)
        self.splineY = si.BSpline(*tckY)

        # and derivatives
        self.dsplineX = self.splineX.derivative()
        self.dsplineY = self.splineY.derivative()

    def set_points(self, pts):
        '''
        Add control points to spline.
        '''
        self.points = pts
        self.centroid = np.mean(self.points, axis=0)
        self.gamma_max = self.points.shape[0] - 1
        self.generate_spline()

    def translate(self, displacement):
        '''
        Translates spline by displacement units.
        '''
        self.points = self.points + displacement
        self.centroid = np.mean(self.points, axis=0)
        self.generate_spline()

    def rotate(self, theta):
        '''
        Rotates spline in theta radians
        '''
        local_points = self.points - self.centroid
        self.points = self.centroid + ( rot(theta) @ local_points.T ).T
        self.generate_spline()

    def get_path_point(self, gamma):
        '''
        Returns the path point at gamma.
        '''
        return np.array([ self.splineX(gamma), self.splineY(gamma) ])
    
    def get_path_gradient(self, gamma):
        '''
        Returns the path gradient at gamma.
        '''
        return np.array([ self.dsplineX(gamma), self.dsplineY(gamma) ])

    def get_path_normal(self, gamma):
        '''
        Returns the path normal vector at gamma.
        '''
        dxd = self.get_path_gradient(gamma)
        normal = self.perp_rot @ dxd
        norm_normal = np.linalg.norm(normal)
        if norm_normal < 0.00000001:
            raise Exception("Gradient vanishes at gamma = " + str(gamma))
        return normal/norm_normal

    def get_gradient_norm(self, gamma):
        dxd = self.get_path_gradient(gamma)
        return np.linalg.norm( dxd )

    def get_path_state(self):
        state = self.system.get_state()
        self.gamma = state[0]
        return self.gamma

    def get_path_control(self):
        return self.dgamma

    def set_path_state(self, gamma):
        self.gamma = gamma
        self.system.set_state(self.gamma)

    def update(self, gamma_ctrl, sample_time):
        self.dgamma = gamma_ctrl
        self.system.set_control(gamma_ctrl) 
        self.system.actuate(sample_time)
        self.get_path_state()
        self.log_path()

    def log_path(self):
        self.logs["gamma"] = self.system.state_log[0]
        self.logs["dgamma"] = self.system.control_log[0]
        return self.logs
    
    def draw_path(self, path_graph, plot_params):
        numpoints = plot_params["numpoints"]
        path_length = plot_params["path_length"]

        xpath, ypath = [], []
        for k in range(numpoints):
            gamma = k*path_length/numpoints
            pos = self.get_path_point(gamma)
            xpath.append(pos[0])
            ypath.append(pos[1])
        path_graph.set_data(xpath, ypath)

    def find_min(self, point, init=0.0):
        '''
        Method for computing minimum distance to point. Returns:
        i) minimizer (parameter solution)
        ii) closest point in the curve
        iii) minimum distance in path
        '''
        def cost(gamma):
            xd = self.get_path_point(gamma)
            return np.norm( xd - point )
        
        results = opt.minimize( cost, init, constraints=opt.LinearConstraint( np.array(1), lb=self.gamma_min, ub=self.gamma_max ) )
        minimizer = results.x
        closest_point = self.get_path_point(minimizer)
        min_distance = cost(minimizer)

        return minimizer, closest_point, min_distance
    
    def compute_distance(self, point):
        '''
        Get minimum distance from this spline to a point.
        '''
        gamma = self.get_path_state()
        gamma_sol, point_sol, distance = self.find_min(point, init=gamma)
        h = ( point - point_sol ).dot( self.normal(gamma_sol) )
        dh = 0.0

        return h, dh