import sys
import json
import importlib
import numpy as np

simulation_config = sys.argv[1].replace(".json","")
sim_module = importlib.import_module("simulations."+simulation_config+".simulation", package=None)

# Simulation loop -------------------------------------------------------------------
T = 20
num_steps = int(T/sim_module.sample_time)
time = np.zeros(num_steps)
print('Running simulation...')
for step in range(0, num_steps):

    # Simulation time
    time[step] = step*sim_module.sample_time

    # CPF controller for the robots
    robot_controls = []
    for k in range(len(sim_module.robots)):
        u = sim_module.controllers[k].get_control()
        robot_controls.append(u)
        sim_module.paths[k] = sim_module.controllers[k].get_path()

    # Updates agent states
    for k in range(len(sim_module.robots)):
        sim_module.robots[k].set_control(robot_controls[k])
        sim_module.robots[k].actuate(sim_module.sample_time)
        
# ----------------------------------------------------------------------------------

# Collect simulation logs ----------------------------------------------------------
robot_logs, control_logs = [], []
for robot in sim_module.robots:
    robot_logs.append( robot.state_log )
    control_logs.append( robot.control_log )

gamma_logs, v_logs = [], []
for path in sim_module.paths:
    gamma_logs.append( path.logs["gamma"] )
    v_logs.append( path.logs["dgamma"] )
# ----------------------------------------------------------------------------------

# Collect simulation logs and save in .json file ------------------------------------
logs = {
    "time": time.tolist(),
    "sample_time": sim_module.sample_time,
    "robots": robot_logs,
    "control": control_logs,
    "gamma": gamma_logs,
    "v_logs": v_logs,
}

with open(simulation_config+".json", "w") as file:
    print("Saving simulation data...")
    json.dump(logs, file, indent=4)