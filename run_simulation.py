import sys, os
import json
import importlib

simulation_config = sys.argv[1].replace(".json","")
sim_module = importlib.import_module("simulations."+simulation_config+".simulation", package=None)

# Simulation loop -------------------------------------------------------------------
T = 15
num_steps = int(T/sim_module.sample_time)
time = []
print('Running simulation...')
for step in range(0, num_steps):

    # Simulation time
    t = step*sim_module.sample_time

    os.system('clear')
    print("Simulating instant t = " + str(float(f'{t:.2f}')) + " s")
    time.append( t )

    # CPF controller for the robots
    robot_controls = []
    for k in range(len(sim_module.robots)):
        u = sim_module.controllers[k].get_control()
        robot_controls.append(u)

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

gamma_logs, v_logs, gamma_barrier_logs = [], [], []
for path in sim_module.paths:
    gamma_logs.append( path.logs["gamma"] )
    v_logs.append( path.logs["dgamma"] )

for controller in sim_module.controllers:
    gamma_barrier_logs.append([])
    for spline_barrier in controller.spline_barriers:
        gamma_barrier_logs[-1].append( spline_barrier.logs["gamma"] )
# ----------------------------------------------------------------------------------

# Collect simulation logs and save in .json file ------------------------------------
logs = {
    "time": time,
    "sample_time": sim_module.sample_time,
    "robots": robot_logs,
    "control": control_logs,
    "gamma": gamma_logs,
    "v_logs": v_logs,
    "gamma_barriers": gamma_barrier_logs
}

with open(simulation_config+".json", "w") as file:
    print("Saving simulation data...")
    json.dump(logs, file, indent=4)