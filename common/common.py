import numpy as np
import scipy.optimize as opt


def rot(angle):
    return np.array([[ np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])


def drot(angle):
    return np.array([[ -np.sin(angle), -np.cos(angle)], [np.cos(angle), -np.sin(angle)]])


class Rect():
    '''
    Simple rectangle.
    '''
    def __init__(self, sides, center_offset):
        self.length = sides[0]
        self.width = sides[1]
        self.center_offset = center_offset

    def get_center(self, pose):
        x, y, angle = pose[0], pose[1], pose[2]
        return ( x - self.center_offset*np.cos(angle), y - self.center_offset*np.sin(angle) )

    def get_corners(self, pose, *args):
        x, y, angle = pose[0], pose[1], pose[2]

        topleft = ( rot(angle) @ np.array([-self.length/2, self.width/2]) + np.array( self.get_center(pose) ) ).tolist()
        topright = ( rot(angle) @ np.array([ self.length/2, self.width/2]) + np.array( self.get_center(pose) ) ).tolist()
        bottomleft = ( rot(angle) @ np.array([-self.length/2, -self.width/2]) + np.array( self.get_center(pose) ) ).tolist()
        bottomright = ( rot(angle) @ np.array([ self.length/2, -self.width/2]) + np.array( self.get_center(pose) ) ).tolist()

        if len(args):
            if 'topleft' in args[0].lower():
                return topleft
            elif 'topright' in args[0].lower():  
                return topright
            elif 'bottomleft' in args[0].lower():
                return bottomleft
            elif 'bottomright' in args[0].lower():
                return bottomright
            else:
                return [ topleft, topright, bottomleft, bottomright ]
        else:
            return [ topleft, topright, bottomleft, bottomright ]


class EllipticalBarrier():
    '''
    Elliptical barrier
    '''
    def __init__(self, shape=[1.0, 1.0]):

        self.center = np.zeros(2)
        self.orientation = 0.0

        self.shape = shape
        self.eigen = np.array([ self.shape[1]**2, self.shape[0]**2 ])
        self.H = rot(self.orientation).T @ np.diag(self.eigen) @ rot(self.orientation)
        self.dH = drot(self.orientation).T @ np.diag(self.eigen) @ rot(self.orientation) + rot(self.orientation).T @ np.diag(self.eigen) @ drot(self.orientation)

    def update_pose(self, new_pose):
        self.center = new_pose[0:2]
        self.orientation = new_pose[2]
        self.H = rot(self.orientation).T @ np.diag(self.eigen) @ rot(self.orientation)
        self.dH = drot(self.orientation).T @ np.diag(self.eigen) @ rot(self.orientation) + rot(self.orientation).T @ np.diag(self.eigen) @ drot(self.orientation)

    def ellipse(self, gamma):
        '''
        Returns the parametrized ellipse describing the barrier boundary.
        '''
        a = self.shape[0]
        b = self.shape[1]
        return np.array([ a*np.cos(gamma)*np.cos(self.orientation) - b*np.sin(gamma)*np.sin(self.orientation) + self.center[0],
                          a*np.cos(gamma)*np.sin(self.orientation) + b*np.sin(gamma)*np.cos(self.orientation) + self.center[1] ])

    def ellipse_gradient(self, gamma):
        '''
        Returns the gradient of the parametrized ellipse (derivative wrt gamma).
        '''
        a = self.shape[0]
        b = self.shape[1]
        return np.array([ -a*np.sin(gamma)*np.cos(self.orientation) - b*np.cos(gamma)*np.sin(self.orientation),
                          -a*np.sin(gamma)*np.sin(self.orientation) + b*np.cos(gamma)*np.cos(self.orientation) ])

    def function(self, value):
        return 0.5 * ( value - self.center ) @ self.H @ ( value - self.center ) - 0.5
    
    def gradient(self, value):
        grad_hij_pi = (-(value - self.center) @ self.H).reshape(2)
        grad_hij_thetai = ( value - self.center ).T @ self.dH @ ( value - self.center )
        return np.hstack( [ grad_hij_pi, grad_hij_thetai ])
    
    def compute_barrier(self, barrier, **kwargs):
        '''
        Computes vehicle barrier (between self and barrier_obj).
        Returns:
        i) barrier value
        ii) barrier gradient
        iii) optimal point on barrier_obj ellipse
        '''
        init = 2*np.pi*np.random.rand()

        if "init" in kwargs.keys():
            init = kwargs["init"]

        def cost(gamma):
            ellipse_point = barrier.ellipse(gamma)
            return self.function( ellipse_point )

        # Search over the neighbor ellipse boundary...
        results = opt.minimize( cost, init, constraints=opt.LinearConstraint( np.array(1), lb=0.0, ub=2*np.pi ) )
        gamma_sol = results.x
        opt_ellipse = barrier.ellipse(gamma_sol)

        barrier_value = self.function(opt_ellipse)
        barrier_gradient = self.gradient(opt_ellipse)

        grad_hij_pj = ((opt_ellipse - self.center) @ self.H).reshape(2)
        grad_hij_thetaj = ( opt_ellipse - self.center ).T @ self.H @ barrier.ellipse_gradient(gamma_sol)
        neighbor_barrier_gradient = np.array([ grad_hij_pj, grad_hij_thetaj ])

        return barrier_value, barrier_gradient, neighbor_barrier_gradient, opt_ellipse
    
    def contour_plot(self, plot_obj, resolution=100):
        '''
        Return the parametrized ellipse graphics.
        '''
        t_series = np.linspace(0, 2*np.pi, resolution)

        xdata, ydata = np.zeros(resolution), np.zeros(resolution)
        for k in range(resolution):
            ellipse_pt = self.ellipse(t_series[k])
            xdata[k], ydata[k] = ellipse_pt[0], ellipse_pt[1]

        plot_obj.set_data( xdata, ydata )