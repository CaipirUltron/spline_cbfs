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
    Class for elliptical barrier. Contains methods for computing the barrier function, gradient and algorithms.
    '''
    def __init__(self, shape=[1.0, 1.0]):

        self.center = np.zeros(2)
        self.orientation = 0.0

        self.shape = shape
        self.eigen = np.array([ 1/(self.shape[0]**2), 1/(self.shape[1]**2) ])
        self.H = rot(self.orientation) @ np.diag(self.eigen) @ rot(self.orientation).T
        self.dH = S @ self.H - self.H @ S
        self.min_theta_grad = 0.05

    def update(self, new_pose):
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
        
        if np.abs(grad_hij_thetai) < self.min_theta_grad:
            grad_hij_thetai = self.min_theta_grad * np.sign(grad_hij_thetai)
        return np.hstack([ grad_hij_pi, grad_hij_thetai ])
    
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
    # Functions for nonlinear transformation --------------------------------
    def nlr_transform_left(x, left_limit):
        return left_limit - np.exp(-x)
    def nlr_transform_middle(x, limits):
        lambda1 = limits[0]
        lambda2 = limits[1]
        return ( lambda1 + lambda2 * np.exp(x) ) / ( 1 + np.exp(x) )
    def nlr_transform_right(x, right_limit):
        return right_limit + np.exp(x)
    # -----------------------------------------------------------------------
    # Inverse function for nonlinear transformation
    def inv_nlr_transform_left(x, left_limit):
        return - np.log( left_limit - x )
    def inv_nlr_transform_middle(x, limits):
        lambda1 = limits[0]
        lambda2 = limits[1]
        return np.log( ( x - lambda1 ) / ( lambda2 - x ) )
    def inv_nlr_transform_right(x, right_limit):
        return np.log( x - right_limit )
    # -----------------------------------------------------------------------
    def __init__(self, A, B):
        '''
        Initialize pencil.
        '''
        self.param = 0.9
        self.update(A, B)

    def update(self, A, B):
        '''
        Updates pencil.
        '''
        dimA = A.shape
        dimB = B.shape
        if dimA != dimB:
            raise Exception("Matrix dimensions are not equal.")
        if (dimA[0] != dimA[1]) or (dimB[0] != dimB[1]):
            raise Exception("Matrices are not square.")
        self._A, self._B = A, B
        self.dim = dimA[0]
        self.compute_eigen()

    def value(self, lambda_param):
        '''
        Returns pencil value for a given lambda.
        '''
        return lambda_param * self._A  - self._B

    def q_function(self, lambda_param, w):
        '''
        Returns the q-function value at lambda and w.
        '''
        H = self.value(lambda_param)
        try:
            wl = np.linalg.inv(H) @ w
            return wl.T @ self._A @ wl
        except np.linalg.LinAlgError:
            return np.inf
    
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
                eig, Q = np.linalg.eig( self.value(self.eigenvalues[k]) )
            else:
                eig, Q = np.linalg.eig( self.schurA_vec[k] * self._A - self.schurB_vec[k] * self._B )
            for i in range(len(eig)):
                if np.abs(eig[i]) <= ZERO_ACCURACY:
                    self.eigenvectors[:,k] = Q[:,i]

        # Sorts eigenpairs
        sorted_args = np.argsort(self.eigenvalues)
        self.eigenvalues = self.eigenvalues[sorted_args]
        self.eigenvectors = self.eigenvectors[:,sorted_args]

    def solve_system(self, w):
        '''
        Given know p.s.d. matrices A, B and vector w,
        compute all the solutions for the system: (lambda A - B) v = w ,
                                                       v.T @ A @ v = 1 .
        '''
        def compute_f(lambda_mod):
            lambdap = transf(lambda_mod)
            return self.q_function( lambdap, w ) - 1

        def compute_qder(lambda_mod):
            lambdap = transf(lambda_mod)
            H = self.value(lambdap)
            Hinv = np.linalg.inv(H)
            vhi = self._A @ Hinv @ w
            return - vhi.T @ Hinv @ vhi

        lambda_candidates, costs = [], []
        for i in range(len(self.eigenvalues)+1):
            if i == 0:
                left_limit = self.eigenvalues[i]
                transf = lambda x : LinearMatrixPencil.nlr_transform_left(x, left_limit)
                inv_transf = lambda x : LinearMatrixPencil.inv_nlr_transform_left(x, left_limit)
                init_guess = self.eigenvalues[i] - self.param/(1-self.param)

                sol = opt.root(compute_f, inv_transf(init_guess), method='lm')
                if sol.success:
                    lambda_candidate = transf( sol.x[0] )
                    lambda_candidates.append( lambda_candidate )
                    costs.append( compute_f(inv_transf( lambda_candidate )) )
            elif i == len(self.eigenvalues):
                right_limit = self.eigenvalues[-1]
                transf = lambda x : LinearMatrixPencil.nlr_transform_right(x, right_limit)
                inv_transf = lambda x : LinearMatrixPencil.inv_nlr_transform_right(x, right_limit)
                init_guess = self.eigenvalues[-1] + self.param/(1-self.param)

                sol = opt.root(compute_f, inv_transf(init_guess), method='lm')
                if sol.success:
                    lambda_candidate = transf( sol.x[0] )
                    lambda_candidates.append( lambda_candidate )
                    costs.append( compute_f(inv_transf( lambda_candidate )) )
            else:
                limits = [ self.eigenvalues[i-1], self.eigenvalues[i] ] 
                transf = lambda x : LinearMatrixPencil.nlr_transform_middle(x, limits)
                inv_transf = lambda x : LinearMatrixPencil.inv_nlr_transform_middle(x, limits)
                
                # Find minimal value in interval (for correct initialization)
                delta_lambda = self.eigenvalues[i] - self.eigenvalues[i-1]
                init_guess = self.eigenvalues[i-1] + self.param * delta_lambda/2
                sol = opt.root(compute_qder, inv_transf(init_guess), method='lm')
                interval_min = sol.x[0]
                
                # Find root
                init_guess_left = interval_min - self.param/(1-self.param)
                init_guess_right = interval_min + self.param/(1-self.param)
                sol_left = opt.root(compute_f, init_guess_left, method='lm')
                sol_right = opt.root(compute_f, init_guess_right, method='lm')
                
                equal = False
                if sol_left.success and sol_right.success:
                    if np.abs( sol_left.x[0] - sol_right.x[0] ) < ZERO_ACCURACY:
                        equal = True
                if equal:
                    lambda_candidate = transf( sol_left.x[0] )
                    lambda_candidates.append( lambda_candidate )
                    costs.append( compute_f(inv_transf( lambda_candidate )) )
                else:
                    if sol_left.success:
                        lambda_candidate = transf( sol_left.x[0] )
                        lambda_candidates.append( lambda_candidate )
                        costs.append( compute_f(inv_transf( lambda_candidate )) )
                    if sol_right.success:
                        lambda_candidate = transf( sol_right.x[0] )
                        lambda_candidates.append( lambda_candidate )
                        costs.append( compute_f(inv_transf( lambda_candidate )) )

        costs = np.array(costs)
        lambda_candidates = np.array(lambda_candidates)

        indexes_sol = np.argwhere(costs < ZERO_ACCURACY)
        lambda_solutions = lambda_candidates[indexes_sol.T].reshape(indexes_sol.shape[0])

        v_solutions = []
        for lambda_solution in lambda_solutions:
            H = self.value( lambda_solution )
            v = np.linalg.inv(H) @ w
            v_solutions.append(v)

        return lambda_solutions, np.array(v_solutions).T

    def __str__(self):         
        '''
        Print the given pencil.
        '''
        np.set_printoptions(precision=3, suppress=True)
        ret_str = '{}'.format(type(self).__name__) + " = \u03BB A - B \n"
        ret_str = ret_str + 'A = ' + self._A.__str__() + '\n'
        ret_str = ret_str + 'B = ' + self._B.__str__()
        return ret_str


class BarrierGrid():
    '''
    Barrier holder class: used to represent connections between a main barrier and its neighbors
    '''
    def __init__(self, barriers=[]):

        self.num_barriers = len(barriers)
        self.barriers = barriers

        self.pencils = np.array([ [None]*self.num_barriers for i in range(self.num_barriers) ])

        for idi in range(self.num_barriers):
            for idj in range(self.num_barriers):
                if idi == idj:
                    continue
                Hi = self.barriers[idi].H
                Hj = self.barriers[idj].H
                self.pencils[idi,idj] = LinearMatrixPencil(Hj, Hi)

    def update_barrier(self, id, new_pose):
        '''
        Update barrier with identifier id and all corresponding pencils.
        '''
        self.barriers[id].update( new_pose )
        for i in range(self.pencils.shape[0]):
            for j in range(self.pencils.shape[1]):
                if i == j:
                    continue
                if i == id:
                    self.pencils[i,j].update( self.pencils[i,j]._A, self.barriers[id].H )
                if j == id:
                    self.pencils[i,j].update( self.barriers[id].H, self.pencils[i,j]._B )

    def compute_barrier(self, idi, idj, pose_i, pose_j):
        '''
        Computes vehicle barrier (between barriers[id1] and barriers[id2] ).
        Returns: i) barrier value, ii) barrier gradient, iii) optimal point on barrier_obj ellipse
        '''
        self.update_barrier(idi, pose_i)
        self.update_barrier(idj, pose_j)

        barrieri = self.barriers[idi]
        barrierj = self.barriers[idj]

        def cost(delta):
            return barrieri.function(delta)

        # Solves system from using method from LinearMatrixPencil class
        Hi = barrieri.H
        pci = np.array(barrieri.center)
        pcj = np.array(barrierj.center)
        wij = Hi @ ( pcj - pci )

        lambda_sols, v_sols = self.pencils[idi,idj].solve_system( wij )
        num_sols = len( lambda_sols )

        # Converts back to solution in terms of delta
        delta_sols = np.zeros([2,num_sols])
        for k in range(num_sols):
            delta_sols[:,k] = v_sols[:,k] + pcj

        # Finds the optimal (global) solution 
        costs = np.zeros(num_sols)
        for k in range(num_sols):
            costs[k] = cost( delta_sols[:,k] )

        # Filters solutions with positive cost and lambda (geometrically incorrect)
        index_sol = np.argmin(costs)

        opt_ellipse = delta_sols[:,index_sol]

        barrier_value = barrieri.function(opt_ellipse)
        barrier_gradient = barrieri.gradient(opt_ellipse)

        # Determine the gamma equivalent solution
        def cost_gamma(gamma):
            return np.linalg.norm( opt_ellipse - barrierj.ellipse(gamma) )
        
        results1 = opt.root( cost_gamma, -np.pi/2, method='lm' )
        results2 = opt.root( cost_gamma,  np.pi/2, method='lm' )

        gamma_costs = np.array([ cost_gamma(results1.x[0]), cost_gamma(results2.x[0]) ])
        results = np.array([ results1.x[0], results2.x[0] ])

        gamma_sol = results[ np.argmin(gamma_costs) ]

        if np.min(gamma_costs) > ZERO_ACCURACY:
            print(np.min(gamma_costs))

        # Neighbor barrier
        ellipse_jac = barrierj.ellipse_jacobian(gamma_sol)

        grad_hij_pj = (opt_ellipse - pci).T @ Hi @ ellipse_jac[:,0:2]
        grad_hij_thetaj = ( opt_ellipse - pci ).T @ Hi @ ellipse_jac[:,2]
        neighbor_barrier_gradient = np.hstack([ grad_hij_pj, grad_hij_thetaj ])
        
        return barrier_value, barrier_gradient, neighbor_barrier_gradient, opt_ellipse
    
    def check_errors(self):
        '''
        Provisory function for checking errors in array of LinearMatrixPencils (all is right)
        '''
        error_matrix = np.zeros([self.num_barriers, self.num_barriers])
        for i in range(self.pencils.shape[0]):
            for j in range(self.pencils.shape[1]):
                if i == j:
                    continue
                error_matrix[i,j] = np.linalg.norm(self.pencils[i,j]._A - self.barriers[j].H) + np.linalg.norm(self.pencils[i,j]._B - self.barriers[i].H)

        return error_matrix