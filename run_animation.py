import sys
import json
import importlib
import matplotlib.pyplot as plt
from graphics import Plot2DSimulation

# Load simulation file
simulation_file = sys.argv[1].replace(".json","")
sim = importlib.import_module("simulations."+simulation_file+".simulation", package=None)

try:
    with open(simulation_file + ".json") as file:
        print("Loading graphical simulation with "+simulation_file + ".json")
        logs = json.load(file)
except IOError:
    print("Couldn't locate "+simulation_file + ".json")

print('Animating simulation...')
plotSim = Plot2DSimulation( sim.robots, sim.paths, logs, plot_config = sim.plot_config )
plotSim.animate()

# plotSim.animation.save(simulation_file + ".mp4", writer=anim.FFMpegWriter(fps=30, codec='h264'), dpi=100)

plt.show()