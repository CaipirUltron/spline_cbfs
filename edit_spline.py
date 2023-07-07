import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from controllers import SplinePath
from spline_editor import SplineEditor

loaded = False

# Create initial spline from given points...
# num_points = 7
# pts_x = [6, -2, 4, 6, 8, 14, 6]
# pts_y = [-3, 2, 5, 0, 5, 2, -3]
# pts = np.array([pts_x, pts_y]).T
# spline_params = { "degree": 3, "points": pts, "orientation": 'left' }

num_points = 7
pts = np.random.rand(num_points,2)
spline_params = { "degree": 3, "points": pts, "orientation": 'left' }

# Or edit from a previously saved configuration
if len(sys.argv) > 2:
    loaded = True
    sim_name = sys.argv[1].replace(".json","")
    spline_name = sys.argv[2].replace(".json","")
    locations = [ "simulations/"+sim_name+"/paths/"+spline_name+".json", "simulations/"+sim_name+"/barriers/"+spline_name+".json" ]
    for location in locations:
        try:
            with open(location,'r') as file:
                print(file)
                print("Loading test: " + sim_name + ".json")
                spline_params = json.load(file)
                spline_params["points"] = np.array( spline_params["points"] )
                break
        except: print("No such file on " + location)

# Generate spline pathy
spline_path = SplinePath( params=spline_params, init_path_state=[0.0] )

# Initialize spline plot
offset = 1

coords = np.hstack([ spline_params["points"][:,0], spline_params["points"][:,1] ])
min = np.min( coords ) - offset
max = np.max( coords ) + offset
plot_params = {
    "axeslim": (min,max,min,max),
    "path_length": num_points + 1, 
    "numpoints": 200
}

fig, axes = plt.subplots(figsize=(6, 6))
axes.set_title('Spline Editor')
axes.set_xlim( plot_params["axeslim"][0:2] )
axes.set_ylim( plot_params["axeslim"][2:4] )
axes.set_aspect('equal', adjustable='box')

spline_graph, = axes.plot([],[], linestyle='dashed', lw=0.8, alpha=0.8, color='b')
editor = SplineEditor(spline_path, spline_graph, plot_params)
plt.show()

# Save new spline configuration
spline_params["points"] = editor.path.points.tolist()

save = False
if loaded:
    save = True
else:
    print("Save file? Y/N")
    if str(input()).lower() == "y":
        save = True
        print("File name: ")
        file_name = str(input())

if save:
    with open(location, "w") as file:
        print("Saving spline at "+location)
        json.dump(spline_params, file, indent=4)