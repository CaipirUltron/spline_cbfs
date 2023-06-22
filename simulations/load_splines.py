import os, sys, json
import numpy as np

simulation_config = sys.argv[1].replace(".json","")

# Limits of scenario
x_coords, y_coords = np.empty(0), np.empty(0)

# Load vehicle paths
path_params = []
path_names = os.listdir("simulations/"+simulation_config+"/paths")
for path_name in path_names:
    with open("simulations/"+simulation_config+"/paths/"+path_name) as file:
        print("Loading: " + path_name)
        path_params.append( json.load(file) )
        pts = np.array( path_params[-1]["points"] )
        path_params[-1]["points"] = pts
        x_coords = np.hstack( [x_coords, pts[:,0] ])
        y_coords = np.hstack( [y_coords, pts[:,1] ])

# Load scenario barriers
barrier_params = []
barrier_names = os.listdir("simulations/"+simulation_config+"/barriers")
for barrier_name in barrier_names:
    with open("simulations/"+simulation_config+"/barriers/"+barrier_name) as file:
        print("Loading: " + barrier_name)
        barrier_params.append( json.load(file) )
        pts = np.array( barrier_params[-1]["points"] )
        barrier_params[-1]["points"] = pts
        x_coords = np.hstack( [x_coords, pts[:,0] ])
        y_coords = np.hstack( [y_coords, pts[:,1] ])

xlimits = [ np.min(x_coords), np.max(x_coords) ]
ylimits = [ np.min(y_coords), np.max(y_coords) ]