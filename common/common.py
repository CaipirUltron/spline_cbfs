import numpy as np
import scipy.optimize as opt
import scipy

ZERO_ACCURACY = 0.0000001

S = np.array([ [0, -1], [1, 0] ])


def rot(angle):
    return np.array([[ np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])


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
        self.eigen = np.array([ 1/(self.shape[0]**2), 1/(self.shape[1]**2) ])
        self.H = rot(self.orientation) @ np.diag(self.eigen) @ rot(self.orientation).T
        self.dH = S @ self.H - self.H @ S

    def update_pose(self, new_pose):
        self.center = new_pose[0:2]
        self.orientation = new_pose[2]
        self.H = rot(self.orientation) @ np.diag(self.eigen) @ rot(self.orientation).T
        self.dH = S @ self.H - self.H @ S

    def ellipse(self, gamma):
        '''
        Returns the parametrized ellipse describing the barrier boundary.
        '''
        a = self.shape[0]
        b = self.shape[1]
        return np.array([ a*np.cos(gamma)*np.cos(self.orientation) - b*np.sin(gamma)*np.sin(self.orientation) + self.center[0],
                          a*np.cos(gamma)*np.sin(self.orientation) + b*np.sin(gamma)*np.cos(self.orientation) + self.center[1] ]).reshape(2)

    def ellipse_jacobian(self, gamma):
        '''
        Returns the Jacobian of the parametrized ellipse (derivative wrt to the robot state).
        '''
        a = self.shape[0]
        b = self.shape[1]

        grad_pos = np.eye(2)
        grad_theta = np.array([ -a*np.cos(gamma)*np.sin(self.orientation) - b*np.sin(gamma)*np.cos(self.orientation),
                                 a*np.cos(gamma)*np.cos(self.orientation) - b*np.sin(gamma)*np.sin(self.orientation) ]).reshape(2,1)
        return np.hstack([ grad_pos, grad_theta ])

    def function(self, value):
        return 0.5 * ( value - self.center ).T @ self.H @ ( value - self.center ) - 0.5
    
    def gradient(self, value):
        grad_hij_pi = (-(value - self.center) @ self.H).reshape(2)
        grad_hij_thetai = 0.5 * ( value - self.center ).T @ self.dH @ ( value - self.center )
        return np.hstack([ grad_hij_pi, grad_hij_thetai ])
    
    def compute_barrier(self, neighbor_barrier, **kwargs):
        '''
        Computes vehicle barrier (between self and barrier_obj).
        Returns:
        i) barrier value
        ii) barrier gradient
        iii) optimal point on barrier_obj ellipse
        '''
        init = self.center

        if "init" in kwargs.keys():
            init = kwargs["init"]

        def constr(delta):
            return neighbor_barrier.function(delta)

        def cost(delta):
            return self.function(delta)

        # Search over the neighbor ellipse...
        # results = opt.minimize( cost, init, constraints=opt.LinearConstraint( np.array(1), lb=-2*np.pi, ub=2*np.pi ) )
        results = opt.minimize( cost, init, constraints=opt.NonlinearConstraint( constr, 0, 0 ) )
        opt_ellipse = results.x

        # Self barrier
        barrier_value = self.function(opt_ellipse)
        barrier_gradient = self.gradient(opt_ellipse)

        # Determine the gamma equivalent solution
        def cost_gamma(gamma):
            return np.linalg.norm( opt_ellipse - neighbor_barrier.ellipse(gamma) )
        
        results1 = opt.minimize( cost_gamma, -np.pi/2, constraints=opt.LinearConstraint( np.array(1), lb=-np.pi, ub=np.pi ) )
        results2 = opt.minimize( cost_gamma,  np.pi/2, constraints=opt.LinearConstraint( np.array(1), lb=0 , ub=np.pi ) )

        cost1 = cost_gamma(results1.x)
        cost2 = cost_gamma(results2.x)

        gamma_sol = results2.x
        if cost1 <= cost2:
            gamma_sol = results1.x

        # Neighbor barrier
        ellipse_jac = neighbor_barrier.ellipse_jacobian(gamma_sol)

        grad_hij_pj = (opt_ellipse - self.center).T @ self.H @ ellipse_jac[:,0:2]
        grad_hij_thetaj = ( opt_ellipse - self.center ).T @ self.H @ ellipse_jac[:,2]
        neighbor_barrier_gradient = np.hstack([ grad_hij_pj, grad_hij_thetaj ])
        
        return barrier_value, barrier_gradient, neighbor_barrier_gradient, opt_ellipse
    
    def contour_plot(self, plot_obj, resolution=50):
        '''
        Return the parametrized ellipse graphics.
        '''
        t_series = np.linspace(0, 2*np.pi, resolution)

        xdata, ydata = np.zeros(resolution), np.zeros(resolution)
        for k in range(resolution):
            ellipse_pt = self.ellipse(t_series[k])
            xdata[k], ydata[k] = ellipse_pt[0], ellipse_pt[1]

        plot_obj.set_data( xdata, ydata )


class LinearMatrixPencil():
    '''
    Class for regular, symmetric linear matrix pencils of the form P(\lambda) = \lambda A - B, where A and B are p.s.d. matrices.
    '''    
    def __init__(self, A, B, **kwargs):

        dimA = A.shape
        dimB = B.shape
        if dimA != dimB:
            raise Exception("Matrix dimensions are not equal.")
        if (dimA[0] != dimA[1]) or (dimB[0] != dimB[1]):
            raise Exception("Matrices are not square.")
        self._A, self._B = A, B
        self.dim = dimA[0]

        self.compute_eigen()

        self.param = 0.5

    def value(self, lambda_param):
        '''
        Returns pencil value.
        '''
        return lambda_param * self._A  - self._B

    def q_function(self, lambda_param, w):
        '''
        Returns the q-function value at lambda and w.
        '''
        H = self.value(lambda_param)
        wl = np.inv(H) @ w
        return wl.T @ self.A @ wl

    def compute_eigen(self):
        '''
        Computes the generalized eigenvalues and eigenvectors of the pencil.
        '''
        # Compute the pencil eigenvalues
        schurA, schurB, _, _, _, _ = scipy.linalg.ordqz(self._B, self._A)
        self.schurA_vec = np.diag(schurA)
        self.schurB_vec = np.diag(schurB)

        self.eigenvalues = np.zeros(self.dim)
        for k in range(self.dim):
            if np.abs(self.schurB_vec[k]) > ZERO_ACCURACY:
                self.eigenvalues[k] = self.schurA_vec[k]/self.schurB_vec[k]
            else:
                self.eigenvalues[k] = np.sign(self.schurA_vec[k]) * np.inf

        # Compute the (normalized, if possible) pencil eigenvectors
        self.eigenvectors = np.zeros([self.dim,self.dim])
        for k in range(len(self.eigenvalues)):
            if np.abs(self.eigenvalues[k]) != np.inf:
                eig, Q = np.linalg.eig( self.value( self.eigenvalues[k]) )
            else:
                eig, Q = np.linalg.eig( self.schurA_vec[k] * self._A - self.schurB_vec[k] * self._B )
            for i in range(len(eig)):
                if np.abs(eig[i]) <= ZERO_ACCURACY:
                    self.eigenvectors[:,k] = Q[:,i]

    def nlr_transform_left(input, left_limit):
        return left_limit - np.exp(-input)

    def nlr_transform_middle(input, limits):
        lambda1 = limits[0]
        lambda2 = limits[1]
        return ( lambda1 + lambda2 * np.exp(input) ) / ( 1 + np.exp(input) )

    def nlr_transform_right(input, right_limit):
        return right_limit + np.exp(input)

    def inv_nlr_transform_left(input, left_limit):
        return - np.log( left_limit - input )

    def inv_nlr_transform_middle(input, limits):
        '''
        Inverse nonlinear transformation used in Newton-Raphson method
        '''
        lambda1 = limits[0]
        lambda2 = limits[1]
        return np.log( ( input - lambda1 ) / ( lambda2 - input ) )

    def inv_nlr_transform_right(input, right_limit):
        return np.log( input - right_limit )

    def solve_system(self, w):
        '''
        Given know p.s.d. matrices A, B and vector w,
        compute all the solutions for the system: (lambda A - B) v = w ,
                                                         v.T @ A v = 1 
        '''
        solutions = []
        for i in range(len(self.eigenvalues)):
            if i == 0:
                transf = self.nlr_transform_left
                inv_transf = self.inv_nlr_transform_left
                init_guess = self.eigenvalues[0] - self.param/(1-self.param)

                sol = opt.root(compute_f, init_guess, method='lm')
                if sol.sucess:
                    solutions.append( inv_transf( sol.x ) )
            elif i == len(self.eigenvalues)-1:
                transf = self.nlr_transform_right
                inv_transf = self.inv_nlr_transform_right
                init_guess = self.eigenvalues[-1] + self.param/(1-self.param)

                sol = opt.root(compute_f, init_guess, method='lm')
                if sol.sucess:
                    solutions.append( inv_transf( sol.x ) )
            else:
                transf = self.nlr_transform_middle
                inv_transf = self.inv_nlr_transform_middle()
                delta_lambda = self.eigenvalues[i] - self.eigenvalues[i-1]
                init_guess1 = self.eigenvalues[i-1] + self.param * delta_lambda / 2
                init_guess2 = self.eigenvalues[i] - self.param * delta_lambda / 2

                sol1 = opt.root(compute_f, init_guess1, method='lm')
                sol2 = opt.root(compute_f, init_guess2, method='lm')
                if sol1.sucess:
                    solutions.append( inv_transf( sol1.x ) )
                if sol2.sucess:
                    solutions.append( inv_transf( sol2.x ) )

        def compute_f(lambda_mod):
            return self.q_function( transf(lambda_mod), w ) - 1
        
        return solutions

    def __str__(self):         
        '''
        Print the given pencil.
        '''
        np.set_printoptions(precision=3, suppress=True)
        ret_str = '{}'.format(type(self).__name__) + " = \u03BB A - B \n"
        ret_str = ret_str + 'A = ' + self._A.__str__() + '\n'
        ret_str = ret_str + 'B = ' + self._B.__str__()
        return ret_str