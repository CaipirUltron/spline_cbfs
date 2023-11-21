import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.patches import Rectangle
from matplotlib import gridspec


class Plot2DSimulation():

    def __init__(self, robots, barrier_grid, paths, spline_barriers, logs, **kwargs):
        '''
        Initializes graphical objects.
        '''        
        self.plot_config = {
            "figsize": (5,5),
            "gridspec": (1,1,1),
            "widthratios": [1],
            "heightratios": [1],
            "axeslim": (-6,6,-6,6),
            "path_length": 100,
            "numpoints": 1000,
            "drawlevel": False,
            "resolution": 50,
            "fps":50,
            "pad":2.0
        }
        if "plot_config" in kwargs.keys():
            self.plot_config = kwargs["plot_config"]

        self.colors = [ 'c', 'm', 'g' ]
        self.robots = robots
        self.barrier_grid = barrier_grid
        self.paths = paths
        self.spline_barriers = spline_barriers

        self.num_robots = len(self.robots)
        self.num_paths = len(self.paths)
        self.num_spline_barriers = len(self.spline_barriers)
        
        # Get logs
        self.sample_time = logs["sample_time"]
        self.time = logs["time"]
        self.robot_logs = logs["robots"]
        self.gamma_logs = logs["gamma"]
        self.gamma_barrier_logs = logs["gamma_barriers"]
        self.priority_logs = logs["priorities"]

        self.num_steps = len(self.time)
        self.markersize = 2.0

        # Initialize plot objects
        self.fig = plt.figure(figsize = self.plot_config["figsize"], constrained_layout=True)
        self.configure()

        self.animation = None

    def configure(self):
        '''
        Configure plot axes
        '''
        self.ax_struct = self.plot_config["gridspec"][0:2]
        width_ratios = self.plot_config["widthratios"]
        height_ratios = self.plot_config["heightratios"]
        gs = gridspec.GridSpec(self.ax_struct[0], self.ax_struct[1], width_ratios = width_ratios, height_ratios = height_ratios)

        # Specify main ax
        main_ax = self.plot_config["gridspec"][-1]

        def order2indexes(k, m):
            i = int((k-1)/m)
            j = int(np.mod(k-1, m))
            return i,j

        if isinstance(main_ax, list):
            i_list, j_list = [], []
            for axes in main_ax:
                i,j = order2indexes(axes, self.ax_struct[1])
                i_list.append(i)
                j_list.append(j)
            i = np.sort(i_list).tolist()
            j = np.sort(j_list).tolist()
            if i[0] == i[-1]: i = i[0]
            if j[0] == j[-1]: j = j[0]
        
        if isinstance(main_ax, int):
            i,j = order2indexes(main_ax, self.ax_struct[1])

        if isinstance(i, int):
            if isinstance(j, int):
                self.main_ax = self.fig.add_subplot(gs[i,j])
            else:
                self.main_ax = self.fig.add_subplot(gs[i,j[0]:(j[-1]+1)])
        else:
            if isinstance(j, int):
                self.main_ax = self.fig.add_subplot(gs[i[0]:(i[-1]+1),j])
            else:
                self.main_ax = self.fig.add_subplot(gs[i[0]:(i[-1]+1),j[0]:(j[-1]+1)])

        axes_lim = self.plot_config["axeslim"]
        self.x_lim = axes_lim[0:2]
        self.y_lim = axes_lim[2:4]

        self.main_ax.set_xlim(*self.x_lim)
        self.main_ax.set_ylim(*self.y_lim)
        # self.main_ax.set_title("", fontsize=18)
        self.main_ax.set_aspect('equal', adjustable='box')

        self.draw_level = self.plot_config["drawlevel"]
        self.fps = self.plot_config["fps"]
        self.numpoints = self.plot_config["resolution"]
        self.path_length = self.plot_config["path_length"]
        self.numpoints = self.plot_config["numpoints"]

        self.create_graphical_objects()

    def create_graphical_objects(self):
        '''
        Initalize graphical objects.
        '''
        self.time_text = self.main_ax.text(self.plot_config["axeslim"][1]-35, self.plot_config["axeslim"][3]-3, str("Time = "))
        self.robot_priority_texts = [ self.main_ax.text(0.0, 0.0, str(self.priority_logs[k][0])) for k in range(self.num_robots) ]

        self.robot_positions, self.robot_trajectories, self.robot_geometries, self.robot_ellipses, self.virtual_pts, self.arrows = [], [], [], [], [], []
        self.ellipse_points = []
        for k in range(self.num_robots):
            robot_pos, = self.main_ax.plot([],[],lw=1,color=self.colors[k],marker='o',markersize=self.markersize)
            self.robot_positions.append(robot_pos)

            ellipse, = self.main_ax.plot([],[],lw=1,color='blue')
            self.robot_ellipses.append(ellipse)

            robot_traj, = self.main_ax.plot([],[],lw=1.5,color=self.colors[k])
            
            robot_x, robot_y, robot_angle = self.robot_logs[k][0][0], self.robot_logs[k][1][0], self.robot_logs[k][2][0]
            center = self.robots[k].geometry.get_center( (robot_x, robot_y, robot_angle) )
            robot_geometry = Rectangle( center,
                                       width=self.robots[k].geometry.length,
                                       height=self.robots[k].geometry.width, 
                                       angle=np.rad2deg(robot_angle), rotation_point="xy" )

            self.robot_geometries.append( robot_geometry )
            self.robot_trajectories.append( robot_traj )

            self.arrows.append([])
            for j in range(self.num_spline_barriers):
                i_arrow, = self.main_ax.plot([],[], linestyle='dashed', lw=1.5, alpha=1.0, color=self.colors[k])
                self.arrows[-1].append( i_arrow )

            ellipse_pt, = self.main_ax.plot([],[],lw=1,color='green',marker='o',markersize=self.markersize)
            self.ellipse_points.append(ellipse_pt)

        self.path_graphs = []
        for k in range(self.num_paths):
            i_graph, = self.main_ax.plot([],[], linestyle='dashed', lw=0.8, alpha=0.8, color=self.colors[k])
            self.path_graphs.append( i_graph )

            virtual_pt,  = self.main_ax.plot([],[],lw=1,color='red',marker='o',markersize=self.markersize)
            self.virtual_pts.append(virtual_pt)

        self.barrier_graphs = []
        for k in range(self.num_spline_barriers):
            i_graph, = self.main_ax.plot([],[], lw=1.0, alpha=1.0, color='r')
            self.barrier_graphs.append( i_graph )

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
        graphical_elements += self.robot_priority_texts
        graphical_elements += self.robot_ellipses
        graphical_elements += self.ellipse_points
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
            self.robot_priority_texts[k].set_position( (self.robot_logs[k][0][i], self.robot_logs[k][1][i]) )
            self.robot_priority_texts[k].set_text(str(self.priority_logs[k][i]))

            xdata, ydata = self.robot_logs[k][0][0:i], self.robot_logs[k][1][0:i]
            self.robot_trajectories[k].set_data(xdata, ydata)

            robot_x, robot_y, robot_angle = self.robot_logs[k][0][i], self.robot_logs[k][1][i], self.robot_logs[k][2][i]
            pose = [robot_x, robot_y, robot_angle]
            self.robots[k].set_state(pose)

            self.robot_geometries[k].xy = self.robots[k].geometry.get_corners(pose, "bottomleft")
            self.robot_geometries[k].angle = np.rad2deg(robot_angle)

            self.main_ax.add_patch(self.robot_geometries[k])

            pck = self.robots[k].get_center_state()
            self.barrier_grid.update_barrier(k, pck)
            self.barrier_grid.barriers[k].contour_plot( self.robot_ellipses[k] )

            for j in range(self.num_robots):
                if k != j:
                    robot_x, robot_y, robot_angle = self.robot_logs[j][0][i], self.robot_logs[j][1][i], self.robot_logs[j][2][i]
                    pose = [robot_x, robot_y, robot_angle]
                    self.robots[j].set_state(pose)
                    pcj = self.robots[j].get_center_state()
                    h, grad_i_h, grad_j_h, ellipse_pt = self.barrier_grid.compute_barrier( k, j, pck, pcj )
            
                    # self.time_text.set_text("barrier " + str([k, j]) + " = " + str(h))
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
        graphical_elements = self.robot_positions + self.robot_geometries + self.robot_trajectories + self.path_graphs + self.barrier_graphs + self.virtual_pts
        graphical_elements.append(self.time_text)
        graphical_elements += self.robot_priority_texts
        graphical_elements += self.robot_ellipses
        graphical_elements += self.ellipse_points
        for k in range(self.num_robots):
            graphical_elements += self.arrows[k]
            
        return graphical_elements

    def animate(self, *args):
        '''
        Show animation starting from specific time (passed as optional argument)
        '''
        initial_time = 0
        if len(args) > 0:
            initial_time = args[0]
        initial_step = int(np.floor(initial_time/self.sample_time))
        self.animation = anim.FuncAnimation(self.fig, func=self.update, init_func=self.init, frames=range(initial_step, self.num_steps), interval=1000/self.fps, repeat=False, blit=True)

    def get_frame(self, t):
        '''
        Returns graphical elements at time t.
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