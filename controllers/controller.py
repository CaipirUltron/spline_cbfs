import numpy as np
from quadratic_program import QuadraticProgram
from dynamic_systems import Unicycle
from common import LinearMatrixPencil, BarrierGroup
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
    def __init__(self, vehicles, lane_barriers, parameters, id):

        self.id = id
        self.path = parameters["path"]
        self.ctrl_dt = parameters["sample_time"]

        # Dimensions and system model initialization
        self.vehicles = vehicles
        self.connectivity = parameters["connectivity"]
        self.num_neighbors = len(self.connectivity)
        self.num_vehicles = self.num_neighbors + 1

        self.system = self.vehicles[self.id]
        self.state_dim = self.system.n
        self.control_dim = self.system.m

        # Lane barriers defining the lane limits
        self.num_lane_barriers = len(lane_barriers)
        self.lane_barriers = []
        for lane in lane_barriers:
            self.lane_barriers.append( copy.deepcopy(lane) )

        # Initialize control parameters
        self.q1, self.q2 = parameters["q1"], parameters["q2"]
        self.alpha = parameters["alpha"]
        self.beta = parameters["beta"]
        self.kappa = parameters["kappa"]
        self.desired_path_speed = parameters["path_speed"]

        # QP for path stabilization
        self.QP1_dim = self.control_dim + 1
        P1 = np.diag(np.hstack([ np.ones(self.control_dim), self.q1 ]))
        q1 = np.zeros(self.QP1_dim)
        self.QP1 = QuadraticProgram(P=P1, q=q1)
        self.u_ctrl = np.zeros(self.control_dim)

        # QP for path dynamics control
        self.QP2_dim = 1 + 1
        self.P2 = np.diag(np.hstack([ 1.0, self.q2 ]))
        q2 = np.hstack([ -self.desired_path_speed, 0.0 ])
        self.QP2 = QuadraticProgram(P=self.P2, q=q2)
        self.dgamma = 0.0

        # Additional parameters for trasient control
        self.toggle_threshold = np.min( self.system.barrier.shape )

        # Filter
        self.ellipse_pt = np.zeros(2)
        self.h = 0.0
        self.Lfh = 0.0
        self.Lgh = np.ones(self.control_dim)

        vehicle_barriers = []
        for vehicle in self.vehicles:
            vehicle_barriers.append( vehicle.barrier )
        self.barrier_group = BarrierGroup(self.id, barriers = vehicle_barriers)

    # def update_pencil(self, id):
    #     '''
    #     Updates pencil relative to id neighbor
    #     '''
    #     Hi = self.system.barrier.H
    #     Hj = self.vehicles[id].barrier.H
    #     self.pencils[id] = LinearMatrixPencil(Hj, Hi)

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
            self.u_ctrl = QP1_sol[0:self.control_dim,]

        # control = np.array([sat(self.u_ctrl[0], limits=[-10, 10]), sat(self.u_ctrl[1], limits=[-10*np.pi, 10*np.pi]) ])
        # control = np.array([sat(self.u_ctrl[0], limits=[-10, 10]), self.u_ctrl[1] ])
        # control = np.array([ self.u_ctrl[0], sat(self.u_ctrl[1], limits=[-10*np.pi, 10*np.pi]) ])
        control = self.u_ctrl

        gamma = self.path.get_path_state()
        xd = self.path.get_path_point(gamma)
        dxd = self.path.get_path_gradient(gamma)
        tilde_x = self.system.get_state()[0:2] - xd
        if np.linalg.norm(tilde_x) >= self.toggle_threshold:
            eta_e = -tilde_x.dot( dxd )
            self.dgamma = -self.kappa*sat(eta_e, limits=[-10.0,10.0])
        else:
            self.dgamma = self.desired_path_speed/np.linalg.norm(dxd)

        # Updates path dynamics
        self.path.update(self.dgamma, self.ctrl_dt)

        return control

# First QP stuff -----------------------------------------------------------------------------------------------------------

    def get_clf_constraint(self):
        '''
        Sets the Lyapunov constraint for path stabilization.
        '''
        # Affine system dynamics
        f = self.system.get_f()
        g = self.system.get_g()

        # Path data
        gamma = self.path.get_path_state()
        xd = self.path.get_path_point(gamma)
        dxd = self.path.get_path_gradient(gamma)
        dgamma = self.path.get_path_control()

        # Lyapunov function candidate
        if type(self.system == Unicycle): 
            tilde_x = self.system.get_state()[0:2] - xd
        else:
            tilde_x = self.system.get_state() - xd
        V = (1/2)*(tilde_x @ tilde_x)
        grad_V = tilde_x

        # Lie derivatives
        if type(self.system == Unicycle): 
            LfV = grad_V.dot( f[0:2] - dxd*dgamma )
            LgV = g[0:2,0:2].T.dot(grad_V)
        else:
            LfV = grad_V.dot( f - dxd*dgamma )
            LgV = g.T.dot(grad_V)
        
        # Stabilization CBF contraints
        a_clf = np.hstack( [ LgV, -1.0 ])
        b_clf = -self.alpha * V - LfV

        return a_clf, b_clf

    def get_vehicle_cbf_constraint(self):
        '''
        Sets the barrier constraint for neighbor vehicles.
        '''
        # Affine system dynamics
        gc = self.system.get_gc()

        # Neighbour barriers for QP1
        a_cbf, b_cbf = [], []
        for id in self.connectivity:
            if id == self.id:
                continue # ignores itself
            
            gc_neighbor = self.vehicles[id].get_gc()
            self.h, grad_i_h, grad_j_h, new_ellipse_pt = self.barrier_group.compute_barrier(id)
            self.Lfh = grad_j_h.T @ gc_neighbor @ self.vehicles[id].get_control()
            self.Lgh = -( grad_i_h.T @ gc )

            # if np.linalg.norm( new_ellipse_pt - self.ellipse_pt ) >= np.min(self.system.barrier.shape):
            #     continue
            # self.ellipse_pt = new_ellipse_pt
            # self.h = h
            # self.Lfh = Lfh
            # self.Lgh = Lgh

            # Adds to the CBF constraints
            a_cbf_k_list = [0 for i in range(self.QP1_dim)]
            a_cbf_k_list[0:self.control_dim] = self.Lgh.tolist()
            a_cbf.append(a_cbf_k_list)
            b_cbf.append( self.beta * self.h + self.Lfh )

        a_cbf = np.array(a_cbf)
        b_cbf = np.array(b_cbf)

        return a_cbf, b_cbf

    def get_lane_cbf_constraint(self):

        # Affine system dynamics
        gc = self.system.get_gc()

        # Lane barriers for QP1
        a_cbf, b_cbf = [], []
        for lane_barrier in self.lane_barriers:
            h, grad_h, _, _ = lane_barrier.compute_barrier( self.system.barrier )

            Lfh = 0.0
            Lgh = (-gc.T @ grad_h).reshape(self.control_dim)

            # Adds to the CBF constraints
            a_cbf_k_list = [0 for i in range(self.QP1_dim)]
            a_cbf_k_list[0:self.control_dim] = Lgh.tolist()
            a_cbf.append(a_cbf_k_list)
            b_cbf.append( self.beta * h + Lfh )

        a_cbf = np.array(a_cbf)
        b_cbf = np.array(b_cbf)

        return a_cbf, b_cbf

# Old stuff -----------------------------------------------------------------------------------------------------------

    def get_forward_constraint(self):
        '''
        Constraint to keep the vehicles moving forward if no conflict exists.
        '''
        a_forward, b_forward = np.array([]).reshape(0,self.QP2_dim), []
        if not self.is_conflicting(self.id):
            a_forward = np.array([ -1.0, 0.0 ])
            b_forward = 0.0
        return a_forward, b_forward