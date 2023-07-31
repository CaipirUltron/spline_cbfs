import sys
import json
import numpy as np
import importlib
import matplotlib.pyplot as plt
from graphics import Plot2DSimulation

plt.rcParams['text.usetex'] = True

simulation_file = sys.argv[1].replace(".json","")
sim = importlib.import_module("simulations."+simulation_file+".simulation", package=None)

try:
    with open(simulation_file + ".json") as file:
        print("Loading graphical simulation with " + simulation_file + ".json")
        logs = json.load(file)
except IOError:
    print("Couldn't locate "+simulation_file + ".json")

font_size = 14
configuration = {
    "figsize": (8,8),
    "gridspec": (2,2,1),
    "widthratios": [1, 1],
    "heightratios": [1.0, 1.0],
    "axeslim": sim.plot_config["axeslim"],
    "path_length": 10,
    "numpoints": 1000,
    "drawlevel": True,
    "resolution": 50,
    "fps": 60,
    "pad": 2.2
}

# Figure 1 ------------------------------------------------------------------------------------

plotSim = Plot2DSimulation( sim.robots, sim.barrier_grid, sim.paths, sim.spline_barriers, logs, plot_config = configuration )
plotSim.fig.suptitle("Safe Autonomous Navigation using a CLF-CBF-based Controller", fontsize=16)
plotSim.fig.tight_layout(pad=configuration["pad"])

# Plot 221
plotSim.plot_config["gridspec"] = (2,2,1)
plotSim.configure()
plotSim.plot_frame(2)

# Subplot 222
plotSim.plot_config["gridspec"] = (2,2,2)
plotSim.configure()
plotSim.plot_frame(4.8)

# Subplot 223
plotSim.plot_config["gridspec"] = (2,2,3)
plotSim.configure()
plotSim.plot_frame(6.5)

# Subplot 224
plotSim.plot_config["gridspec"] = (2,2,4)
plotSim.configure()
plotSim.plot_frame(8.5)

plt.savefig("trajectories.eps", format='eps', transparent=True)

# Figure 2 ------------------------------------------------------------------------------------

fig = plt.figure(constrained_layout=True)
time = logs["time"]
max_time = 10

num_robots = len(sim.robots)
ax_state, ax_control = [], []
for k in range(num_robots):
    robot_number = k+1
    gamma = logs["gamma"][k]

    state_x = logs["robots"][k][0]
    state_y = logs["robots"][k][1]

    error_x, error_y = [], []
    for i in range(len(gamma)):
        pt = sim.paths[k].get_path_point( gamma[i] )
        error_x.append( state_x[i] - pt[0] )
        error_y.append( state_y[i] - pt[1] )
    all_errors = np.hstack([error_x, error_y])

    control_v = logs["control"][k][0]
    control_w = logs["control"][k][1]
    all_controls = np.hstack([control_v, control_w])

    ax_state.append( fig.add_subplot(int(str(num_robots)+"2"+str(robot_number+k))) )
    # ax_state[-1].set_aspect('equal', adjustable='box')
    ax_state[-1].set_title('Vehicle '+str(robot_number)+' path following error', fontsize=font_size)
    ax_state[-1].plot(time, error_x, "--", label='$e_{i,x} [m]$', linewidth=2, markersize=10)
    ax_state[-1].plot(time, error_y, "--", label='$e_{i,y} [m]$', linewidth=2, markersize=10)
    ax_state[-1].legend(fontsize=11, loc='upper right')
    ax_state[-1].set_xlim(0, max_time)
    ax_state[-1].set_ylim(np.min(all_errors)-1, np.max(all_errors)+1)
    if k == num_robots - 1:
        ax_state[-1].set_xlabel('Time [s]', fontsize=14)
    plt.grid()

    ax_control.append( fig.add_subplot(int(str(num_robots)+"2"+str(robot_number+k+1))) )
    # ax_control[-1].set_aspect('equal', adjustable='box')
    ax_control[-1].set_title('Vehicle '+str(robot_number)+' control', fontsize=font_size)
    ax_control[-1].plot(time, control_v, "--", label='$v [m/s]$', linewidth=2, markersize=10)
    ax_control[-1].plot(time, control_w, "--", label='$\omega [rad/s]$', linewidth=2, markersize=10)
    ax_control[-1].legend(fontsize=11, loc='lower right')
    ax_control[-1].set_xlim(0, max_time)
    ax_control[-1].set_ylim(np.min(all_controls)-1, np.max(all_controls)+1)
    if k == num_robots - 1:
        ax_control[-1].set_xlabel('Time [s]', fontsize=14)
    plt.grid()

plt.savefig("signals.eps", format='eps', transparent=True)

plt.show()