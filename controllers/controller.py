import numpy as np
from quadratic_program import QuadraticProgram
from dynamic_systems import Unicycle


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
    def __init__(self, vehicles, parameters, id):

        self.id = id
        self.path = parameters["path"]
        self.ctrl_dt = parameters["sample_time"]

        # Dimensions and system model initialization
        self.vehicles = vehicles
        self.num_vehicles = len(self.vehicles)

        self.system = self.vehicles[self.id]
        self.state_dim = self.system.n
        self.control_dim = self.system.m

        # Lane barriers defining the lane limits
        # self.lane_barriers = lane_barriers

        # Initialize parameters
        self.q1, self.q2 = parameters["q1"], parameters["q2"]
        self.alpha1, self.alpha2 = parameters["alpha1"], parameters["alpha2"]
        self.beta0 = parameters["beta0"]
        self.radius = parameters["radius"]
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
        self.k_e = 0.1

        # Additional parameters for trasient control
        self.toggle_threshold = 1.0

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

    def get_control(self, vehicles, lane_barriers):
        '''
        Computes the QP-CPF control.
        '''
        # Constraints for QP1
        a_clf, b_clf = self.get_clf_constraint()

        a_vehicle_cbf, b_vehicle_cbf = self.get_vehicle_cbf_constraint(vehicles)
        a_lane_cbf, b_lane_cbf = self.get_vehicle_cbf_constraint(lane_barriers)

        if len(b_vehicle_cbf) == 0:
            a_vehicle_cbf = np.array([]).reshape(0,self.QP1_dim)
            b_vehicle_cbf = []

        if len(b_lane_cbf) == 0:
            a_lane_cbf = np.array([]).reshape(0,self.QP1_dim)
            b_lane_cbf = []

        A1 = np.vstack([ a_clf, a_vehicle_cbf, a_lane_cbf ])
        b1 = np.hstack([ b_clf, b_vehicle_cbf, b_lane_cbf ])

        # Solve QP1 and get vehicle control
        self.QP1.set_inequality_constraints(A1, b1)
        QP1_sol = self.QP1.get_solution()

        if not QP1_sol is None:
            self.u_ctrl = QP1_sol[0:self.control_dim,]

        control = np.array([sat(self.u_ctrl[0], limits=[-10, 10]), sat(self.u_ctrl[1], limits=[-np.pi, np.pi]) ])
        # control = self.u_ctrl

        gamma = self.path.get_path_state()
        xd = self.path.get_path_point(gamma)
        dxd = self.path.get_path_gradient(gamma)
        tilde_x = xd - self.system.get_state()[0:2]
        if np.linalg.norm(tilde_x) >= self.toggle_threshold:
            eta_e = tilde_x.dot( dxd )
            self.dgamma = -sat(eta_e, limits=[-10.0,10.0])
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
            LfV = grad_V.dot( f[0:2] - dxd*dgamma   )
            LgV = g[0:2,0:2].T.dot(grad_V)
        else:
            LfV = grad_V.dot( f - dxd*dgamma   )
            LgV = g.T.dot(grad_V)
        
        # Stabilization CBF contraints
        a_clf = np.hstack( [ LgV, -1.0 ])
        b_clf = -self.alpha1 * V - LfV

        return a_clf, b_clf

    def get_vehicle_cbf_constraint(self, vehicles):
        '''
        Sets the barrier constraint for neighbor vehicles.
        '''
        # Affine system dynamics
        f = self.system.get_f()
        g = self.system.get_g()

        # Neighbour barriers for QP1
        a_cbf, b_cbf = [], []
        for neighbour_id in range(self.num_vehicles):
            if neighbour_id != self.id:
                f_neighbor = vehicles[neighbour_id].get_f()
                g_neighbor = vehicles[neighbour_id].get_g()

                delx = self.system.get_state() - vehicles[neighbour_id].get_state()
                delx_normalized = delx/np.linalg.norm(delx)
                Distance = np.linalg.norm(delx)

                h = (Distance - 2*self.radius)
                Lfh = delx_normalized.dot( f - f_neighbor - g_neighbor @ vehicles[neighbour_id].get_control() )
                Lgh = g.T @ delx_normalized

                # Adds to the CBF constraints
                a_cbf_k_list = [0 for i in range(self.QP1_dim)]
                a_cbf_k_list[0:self.control_dim] = -Lgh
                a_cbf.append(a_cbf_k_list)
                b_cbf.append( self.beta0 * h + Lfh )

        a_cbf = np.array(a_cbf)
        b_cbf = np.array(b_cbf)

        return a_cbf, b_cbf

    def get_lane_cbf_constraint(self, lane_barriers):

        # Affine system dynamics
        f = self.system.get_f()
        g = self.system.get_g()

        p = self.system.get_state()

        # Lane barriers for QP1
        a_cbf, b_cbf = [], []
        for lane_barrier in range(self.lane_barriers):
            gamma = self.get_path_state()
            gamma_sol, point_sol, distance = self.find_min(p, init=gamma)
            h = ( p - point_sol ).dot( self.normal(gamma_sol) )
            dh = 0.0


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