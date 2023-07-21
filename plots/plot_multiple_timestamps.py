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
    "pad": 3.0
}

plotSim = Plot2DSimulation( sim.robots, sim.barrier_grid, sim.paths, sim.spline_barriers, logs, plot_config = configuration )
plotSim.fig.suptitle("Traffic Control using CLF-CBFs", fontsize=16)
plotSim.fig.tight_layout(pad=configuration["pad"])

# Plot 221
plotSim.plot_config["gridspec"] = (2,2,1)
plotSim.configure()
plotSim.plot_frame(0.5)

# Subplot 222
plotSim.plot_config["gridspec"] = (2,2,2)
plotSim.configure()
plotSim.plot_frame(1.5)

# Subplot 223
plotSim.plot_config["gridspec"] = (2,2,3)
plotSim.configure()
plotSim.plot_frame(4.0)

# Subplot 224
plotSim.plot_config["gridspec"] = (2,2,4)
plotSim.configure()
plotSim.plot_frame(8.0)

# time = logs["time"]

# state_x = logs["state"][0]
# state_y = logs["state"][1]
# all_states = np.hstack([state_x, state_y])

# control_x = logs["control"][0]
# control_y = logs["control"][1]
# all_controls = np.hstack([control_x, control_y])

# max_time = 10

# # Subplot 325
# ax1 = plotSim.fig.add_subplot(325)
# # ax1.set_aspect('equal', adjustable='box')
# ax1.set_title('State', fontsize=font_size)
# ax1.plot(time, state_x, "--", label='$x_1$', linewidth=2, markersize=10)
# ax1.plot(time, state_y, "--", label='$x_2$', linewidth=2, markersize=10)
# ax1.legend(fontsize=14, loc='upper right')
# ax1.set_xlim(0, max_time)
# ax1.set_ylim(np.min(all_states)-1, np.max(all_states)+1)
# ax1.set_xlabel('Time [s]', fontsize=14)
# # plt.grid()

# # Subplot 326
# ax2 = plotSim.fig.add_subplot(326)
# # ax2.set_aspect('equal', adjustable='box')
# ax2.set_title('Control', fontsize=font_size)
# ax2.plot(time, control_x, "--", label='$u_1$', linewidth=2, markersize=10, alpha=1.0)
# ax2.plot(time, control_y, "--", label='$u_2$', linewidth=2, markersize=10, alpha=0.6) 
# ax2.legend(fontsize=14, loc='upper right')
# ax2.set_xlim(0, max_time)
# ax2.set_ylim(np.min(all_controls)-1, np.max(all_controls)+1)
# ax2.set_xlabel('Time [s]', fontsize=14)
# # plt.grid()

plt.savefig(simulation_file + ".eps", format='eps', transparent=True)
# plt.savefig(simulation_file + ".svg", format="svg",transparent=True)

plt.show()