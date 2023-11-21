import numpy as np
from quadratic_program import QuadraticProgram
from dynamic_systems import Unicycle
from common import rot
import copy

def sat(u, limits):
    '''
    Scalar saturation.
    '''
    min = limits[0]
    max = limits[1]
    if u > max:
        return max
    if u < min:
        return min
    return u


class PFController:
    '''
    This class implements a simple path following controller.
    '''
    def __init__(self, vehicles, barrier_grid, spline_barriers, parameters, id):

        self.id = id
        self.path = parameters["path"]
        self.ctrl_dt = parameters["sample_time"]

        # Dimensions and system model initialization
        self.vehicles = vehicles
        self.connectivity = parameters["connectivity"]
        self.num_neighbors = len(self.connectivity)
        self.num_vehicles = self.num_neighbors + 1

        self.vehicle = self.vehicles[self.id]
        self.state_dim = self.vehicle.n
        self.control_dim = self.vehicle.m

        self.barrier_grid = barrier_grid

        # Spline barriers defining the lane limits
        self.num_spline_barriers = len(spline_barriers)
        self.spline_barriers = []
        for lane in spline_barriers:
            self.spline_barriers.append( copy.deepcopy(lane) )

        # Initialize control parameters
        self.P = np.diag([10.0, 1.0])
        self.alpha, self.beta1, self.beta2, self.kappa = parameters["alpha"], parameters["beta1"], parameters["beta2"], parameters["kappa"]
        self.desired_path_speed = parameters["path_speed"]

        # QP for path stabilization
        self.QP1_dim = self.control_dim + 1
        P1 = np.diag(np.hstack([ np.ones(self.control_dim), parameters["q1"] ]))
        self.QP1 = QuadraticProgram(P=P1, q=np.zeros(self.QP1_dim))
        self.u_ctrl = np.zeros(self.control_dim)

        # Additional parameters for trasient control
        self.toggle_threshold = np.max( self.barrier_grid.barriers[self.id].shape )
        # self.toggle_threshold = 5
        self.Lgh_threshold = 0.001

        # Filter
        self.ellipse_pt = np.zeros(2)
        self.h = 0.0
        self.Lfh = 0.0
        self.Lgh = np.zeros(self.control_dim)

    def set_path_speed(self, vd):
        '''
        Sets the desired speed for the path variable.
        '''
        self.desired_path_speed = vd
        print("Desired speed set to " + str(self.desired_path_speed))

    def get_path(self):
        '''
        Returns the controller path.
        '''
        return self.path

    def get_control(self):
        '''
        Computes the QP-CPF control.
        '''
        # Constraints for QP1
        a_clf, b_clf = self.get_clf_constraint()

        a_vehicle_cbf, b_vehicle_cbf = self.get_vehicle_cbf_constraint()
        a_lane_cbf, b_lane_cbf = self.get_lane_cbf_constraint()

        if len(b_vehicle_cbf) == 0:
            a_vehicle_cbf = np.array([]).reshape(0,self.QP1_dim)
            b_vehicle_cbf = []

        if len(b_lane_cbf) == 0:
            a_lane_cbf = np.array([]).reshape(0,self.QP1_dim)
            b_lane_cbf = []

        A1 = np.vstack([ a_clf, a_vehicle_cbf, a_lane_cbf ])
        b1 = np.hstack([ b_clf, b_vehicle_cbf, b_lane_cbf ])

        # A1 = np.vstack([ a_clf, a_vehicle_cbf ])
        # b1 = np.hstack([ b_clf, b_vehicle_cbf ])

        # Solve QP1 and get vehicle control
        self.QP1.set_inequality_constraints(A1, b1)
        QP1_sol = self.QP1.get_solution()

        if not QP1_sol is None:
            self.u_ctrl = QP1_sol[0:self.control_dim]

        control = np.array([sat(self.u_ctrl[0], limits=[-1, 10]), sat(self.u_ctrl[1], limits=[-2*np.pi, 2*np.pi]) ])
        # control = np.array([sat(self.u_ctrl[0], limits=[-10, 10]), self.u_ctrl[1] ])
        # control = np.array([ self.u_ctrl[0], sat(self.u_ctrl[1], limits=[-10*np.pi, 10*np.pi]) ])
        # control = self.u_ctrl

        gamma = self.path.get_path_state()
        xd = self.path.get_path_point( gamma )
        dxd = self.path.get_path_gradient( gamma )

        tilde_x = self.vehicle.get_state()[0:2] - xd
        eta_e = tilde_x.dot( dxd )
        if np.linalg.norm(tilde_x) >= self.toggle_threshold and eta_e < 0:
            self.dgamma = - sat(eta_e, limits=[-self.kappa,0.0]) * gamma
            # self.dgamma = 0.0
        else:
            self.dgamma = self.desired_path_speed/np.linalg.norm( dxd )
        
        # self.dgamma = self.desired_path_speed/np.linalg.norm(dxd)

        # Updates path dynamics
        self.path.update(self.dgamma, self.ctrl_dt)

        return control

# First QP stuff -----------------------------------------------------------------------------------------------------------

    def get_clf_constraint(self):
        '''
        Sets the Lyapunov constraint for path stabilization.
        '''
        # Affine system dynamics
        f = self.vehicle.get_f()
        g = self.vehicle.get_g()

        # Path data
        gamma = self.path.get_path_state()
        xd = self.path.get_path_point(gamma)
        dxd = self.path.get_path_gradient(gamma)
        dgamma = self.path.get_path_control()

        # Lyapunov function candidate
        if type(self.vehicle == Unicycle):
            pi = self.vehicle.get_state()[0:2]
            theta = self.vehicle.get_state()[2]
            R = rot(theta)
            tilde_x = R.T @ ( pi - xd )
        else:
            tilde_x = self.vehicle.get_state() - xd

        V = (1/2)*(tilde_x @ self.P @ tilde_x)
        grad_V = tilde_x @ self.P

        # Lie derivatives
        if type(self.vehicle == Unicycle):
            LfV = grad_V @ R.T @ ( - dxd*dgamma )
            bar_g = g[:2,:2]
            bar_g[:,1] += np.array([[0, 1],[-1, 0]]) @ ( pi - xd )
            LgV = grad_V @ R.T @ bar_g
        else:
            LfV = grad_V @ ( f - dxd*dgamma )
            LgV = grad_V @ g
        
        # Stabilization CBF contraints
        a_clf = np.hstack( [ LgV, -1.0 ])
        b_clf = -self.alpha * V - LfV

        return a_clf, b_clf

    def get_vehicle_cbf_constraint(self):
        '''
        Sets the barrier constraint for neighbor vehicles.
        '''
        # Affine system dynamics
        gc = self.vehicle.get_gc()
        center_state = self.vehicle.get_center_state()

        # Neighbour barriers for QP1
        a_cbf, b_cbf = [], []
        for id in self.connectivity:
            if id == self.id:
                continue

            neighbor_vehicle = self.vehicles[id]
            gc_neighbor = neighbor_vehicle.get_gc()

            center_state_neighbor = neighbor_vehicle.get_center_state()
            self.h, grad_i_h, grad_j_h, el_pt = self.barrier_grid.compute_barrier(self.id, id, center_state, center_state_neighbor)

            self.Lfh = ( grad_j_h @ gc_neighbor ) @ neighbor_vehicle.get_control()
            self.Lgh = ( grad_i_h @ gc )

            # Adds to the CBF constraints
            a_cbf_k_list = [0 for i in range(self.QP1_dim)]
            a_cbf_k_list[0:self.control_dim] = (-self.Lgh).tolist()
            a_cbf.append(a_cbf_k_list)
            b_cbf.append( self.beta1 * self.h + self.Lfh )

        a_cbf = np.array(a_cbf)
        b_cbf = np.array(b_cbf)

        return a_cbf, b_cbf

    def get_lane_cbf_constraint(self):

        # Affine system dynamics
        gc = self.vehicle.get_gc()

        # Lane barriers for QP1
        a_cbf, b_cbf = [], []
        for spline_barrier in self.spline_barriers:
            h, grad_h, _, _ = spline_barrier.compute_barrier( self.barrier_grid.barriers[self.id] )

            Lfh = 0.0
            Lgh = ( grad_h @ gc )

            # Adds to the CBF constraints
            a_cbf_k_list = [0 for i in range(self.QP1_dim)]
            a_cbf_k_list[0:self.control_dim] = (-Lgh).tolist()
            a_cbf.append(a_cbf_k_list)
            b_cbf.append( self.beta2 * h + Lfh )

        a_cbf = np.array(a_cbf)
        b_cbf = np.array(b_cbf)

        return a_cbf, b_cbf