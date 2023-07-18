import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.patches import Rectangle


class Plot2DSimulation():

    def __init__(self, robots, barrier_grid, paths, spline_barriers, logs, **kwargs):
        '''
        Initializes graphical objects.
        '''        
        plot_config = {
            "axeslim": (-6,6,-6,6),
            "path_length": 40, 
            "numpoints": 80,
            "fps": 60
        }
        if "plot_config" in kwargs.keys():
            plot_config = kwargs["plot_config"]

        self.colors = [ 'c', 'm', 'g' ]
        self.robots = robots
        self.barrier_grid = barrier_grid
        self.paths = paths
        self.spline_barriers = spline_barriers

        self.num_robots = len(self.robots)
        self.num_paths = len(self.paths)
        self.num_spline_barriers = len(self.spline_barriers)
        
        # Initialize plot objects
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Traffic Control with CLF-CBFs')

        self.ax_lims = plot_config["axeslim"][0:2]
        self.ax.set_xlim( plot_config["axeslim"][0:2] )
        self.ax.set_ylim( plot_config["axeslim"][2:4] )
        self.ax.set_aspect('equal', adjustable='box')

        # Get logs
        self.sample_time = logs["sample_time"]
        self.time = logs["time"]
        self.robot_logs = logs["robots"]
        self.gamma_logs = logs["gamma"]
        self.gamma_barrier_logs = logs["gamma_barriers"]

        self.num_steps = len(self.time)

        # Get point resolution for graphical objects
        self.path_length = plot_config["path_length"]
        self.numpoints = plot_config["numpoints"]
        self.fps = plot_config["fps"]

        # Initalize graphical objects
        self.time_text = self.ax.text(plot_config["axeslim"][1]-35, plot_config["axeslim"][3]-3, str("Time = "))

        self.robot_positions, self.robot_trajectories, self.robot_geometries, self.robot_ellipses, self.virtual_pts, self.arrows = [], [], [], [], [], []
        self.ellipse_points = []
        for k in range(self.num_robots):
            robot_pos, = self.ax.plot([],[],lw=1,color=self.colors[k],marker='o',markersize=4.0)
            self.robot_positions.append(robot_pos)

            ellipse, = self.ax.plot([],[],lw=1,color='blue')
            self.robot_ellipses.append(ellipse)

            robot_traj, = self.ax.plot([],[],lw=2,color=self.colors[k])
            
            robot_x, robot_y, robot_angle = self.robot_logs[k][0][0], self.robot_logs[k][1][0], self.robot_logs[k][2][0]
            center = self.robots[k].geometry.get_center( (robot_x, robot_y, robot_angle) )
            robot_geometry = Rectangle( center,
                                       width=self.robots[k].geometry.length,
                                       height=self.robots[k].geometry.width, 
                                       angle=np.rad2deg(robot_angle), rotation_point="xy" )

            self.robot_trajectories.append( robot_traj )
            self.robot_geometries.append( robot_geometry )

            self.arrows.append([])
            for j in range(self.num_spline_barriers):
                i_arrow, = self.ax.plot([],[], linestyle='dashed', lw=1.5, alpha=1.0, color=self.colors[k])
                self.arrows[-1].append( i_arrow )

            ellipse_pt, = self.ax.plot([],[],lw=1,color='green',marker='o',markersize=4.0)
            self.ellipse_points.append(ellipse_pt)

        self.path_graphs = []
        for k in range(self.num_paths):
            i_graph, = self.ax.plot([],[], linestyle='dashed', lw=0.8, alpha=0.8, color=self.colors[k])
            self.path_graphs.append( i_graph )

            virtual_pt,  = self.ax.plot([],[],lw=1,color='red',marker='o',markersize=4.0)
            self.virtual_pts.append(virtual_pt)

        self.barrier_graphs = []
        for k in range(self.num_spline_barriers):
            i_graph, = self.ax.plot([],[], lw=1.5, alpha=1.0, color='r')
            self.barrier_graphs.append( i_graph )

        self.animation = None

    def init(self):
        '''
        Initialization function for the animation.
        '''
        self.time_text.text = str("Time = ")

        for i in range(self.num_robots):
            self.robot_positions[i].set_data([],[])
            self.robot_trajectories[i].set_data([],[])
            self.robot_ellipses[i].set_data([],[])

        for i in range(self.num_paths):
            xpath, ypath = [], []
            for k in range(self.numpoints):
                gamma = k*self.path_length/self.numpoints
                pos = self.paths[i].get_path_point(gamma)
                xpath.append(pos[0])
                ypath.append(pos[1])
            self.path_graphs[i].set_data(xpath, ypath)
            self.virtual_pts[i].set_data([],[])

        for i in range(self.num_spline_barriers):
            xpath, ypath = [], []
            for k in range(self.numpoints):
                gamma = k*self.path_length/self.numpoints
                # self.paths[i].set_path_state(gamma)
                if gamma <= self.spline_barriers[i].gamma_max:
                    pos = self.spline_barriers[i].get_path_point(gamma)
                    xpath.append(pos[0])
                    ypath.append(pos[1])
                else: break
            self.barrier_graphs[i].set_data(xpath, ypath)

        graphical_elements = self.robot_positions + self.robot_trajectories + self.path_graphs + self.barrier_graphs + self.virtual_pts + self.ellipse_points
        graphical_elements.append(self.time_text)
        for k in range(self.num_robots):
            graphical_elements += self.arrows[k]
        
        return graphical_elements

    def update(self, i):
        '''
        Update function for the animation.
        '''
        # Update simulation time graphics
        current_time = np.around(self.time[i], decimals = 2)
        self.time_text.set_text("Time = " + str(current_time) + "s")

        # Update robot graphics
        for k in range(self.num_robots):
            self.robot_positions[k].set_data(self.robot_logs[k][0][i], self.robot_logs[k][1][i])

            xdata, ydata = self.robot_logs[k][0][0:i], self.robot_logs[k][1][0:i]
            self.robot_trajectories[k].set_data(xdata, ydata)

            robot_x, robot_y, robot_angle = self.robot_logs[k][0][i], self.robot_logs[k][1][i], self.robot_logs[k][2][i]
            pose = (robot_x, robot_y, robot_angle)
            self.robot_geometries[k].xy = self.robots[k].geometry.get_corners(pose, "bottomleft")
            self.robot_geometries[k].angle = np.rad2deg(robot_angle)

            self.ax.add_patch(self.robot_geometries[k])

            center_pose = np.hstack([ self.robots[k].geometry.get_center(pose), robot_angle ])

            self.barrier_grid.update_barrier( k, center_pose )
            self.barrier_grid.barriers[k].contour_plot( self.robot_ellipses[k] )

            for j in range(self.num_robots):
                if k != j:
                    robot_x, robot_y, robot_angle = self.robot_logs[j][0][i], self.robot_logs[j][1][i], self.robot_logs[j][2][i]
                    pose = (robot_x, robot_y, robot_angle)
                    center_pose = np.hstack([ self.robots[j].geometry.get_center(pose), robot_angle ])
                    self.barrier_grid.update_barrier( j, center_pose )
                    h, grad_i_h, grad_j_h, ellipse_pt = self.barrier_grid.compute_barrier( k, j )
                    # self.time_text.set_text("dH " + str([k, j]) + " = " + str( dH ))
                    
                    self.time_text.set_text("barrier " + str([k, j]) + " = " + str(h))
                    self.ellipse_points[j].set_data(ellipse_pt[0], ellipse_pt[1])

            # Update barrier graphics (arrows)
            for j in range(self.num_spline_barriers):
                gamma = self.gamma_barrier_logs[k][j][i]
                self.spline_barriers[j].set_path_state(gamma)
                normal = self.spline_barriers[j].get_path_normal(gamma)
                _, _, spline_pt, _ = self.spline_barriers[j].compute_barrier( self.barrier_grid.barriers[k] )
                self.arrows[k][j].set_data([ spline_pt[0], spline_pt[0] + normal[0]], [ spline_pt[1], spline_pt[1] + normal[1]])

        # Update path graphics
        for k in range(self.num_paths):
            if len(self.gamma_logs[k]) > 0:
                gamma = self.gamma_logs[k][i]
                pos = self.paths[k].get_path_point(gamma)
                self.virtual_pts[k].set_data(pos[0], pos[1])

        # Add artists
        graphical_elements = self.robot_positions + self.robot_trajectories + self.robot_geometries + self.path_graphs + self.barrier_graphs + self.virtual_pts
        graphical_elements.append(self.time_text)
        graphical_elements += self.robot_ellipses
        graphical_elements += self.ellipse_points
        for k in range(self.num_robots):
            graphical_elements += self.arrows[k]
            
        return graphical_elements

    def animate(self):
        '''
        Show animation.
        '''
        self.animation = anim.FuncAnimation(self.fig, func=self.update, init_func=self.init, frames=self.num_steps, interval=1000/self.fps, repeat=False, blit=True)
        plt.show()

    def get_frame(self, t):
        '''
        Updates frame at time time.
        '''
        self.init()
        step = int(np.floor(t/self.sample_time))
        graphical_elements = self.update(step)

        return graphical_elements

    def plot_frame(self, t):
        '''
        Plots specific animation frame at time t.
        '''
        self.get_frame(t)
        plt.show()