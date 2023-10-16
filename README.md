
![logo_withebg](https://github.com/gandradeng/spline_cbfs/assets/25594150/13ccd670-9abe-45f2-8572-19c5d7027b18)


# BSpline-based Path Following using CLF and CBFs


This project consists of the implementation of a traffic simulator for autonomous vehicles using a safety-critical path-following controller.
The controller is based on the CLF-CBF framework and is able to steer the vehicles towards spline-based paths while avoiding collisions.
Published in the Sixth Iberian Robotics Conference - [ROBOT2023](https://robot2023.isr.uc.pt/)
## Installation:
In root directory: 
```
$ pip install -e .
```
## Running Simulation: 
Default simulation runs with two vehicles in the pre-defined paths and lane splines. To run the numerical simulation type:
```
$ python3 run_simulation.py simulationX
```
**X** must be replaced by the simulation scenario located in ./simulations/simulationX. In the current version, *simulation1* and *simulation2* scenarios are avalaible for testing. 

## Running Animation:
After the simulationX is finished, to run the animation type:
```
$ python3 run_animation.py simulationX
```
This will run an animation with the previous simulation.


## Editing Lanes and Paths:
It is also possible to edit splines with **edit_spline.py** by typing:
```
$ python3 edit_spline.py simulationX splineY 
```
 This opens spline editor for a splineY inside the folder *./simulations/simulationX*.

## Adding Vehicles:
1. To add a new vehicle you must also create a new path. For example, you can go to *./simulations/simulation1/paths/* and copy the file path.json to a new one, like pathN.json (for the N-th vehicle). Then you can edit pathN using the spline editor.
2. Then go to /simulations/simulation**X**/simulation.py (replace X by the desired scenario) and:
  - Add a new Unicycle robotN (for the N-th robot) object after line [10](https://github.com/gandradeng/spline_cbfs/blob/7538ee0b9ca8082b2aa1eeda8b5982a17ac5bee0/simulations/simulation1/simulation.py#L10) (copy and change the existing one);
  - Add the created robot object at [line 12](https://github.com/gandradeng/spline_cbfs/blob/7538ee0b9ca8082b2aa1eeda8b5982a17ac5bee0/simulations/simulation1/simulation.py#L12) list;
  - Create another connectivity element at line [14](https://github.com/gandradeng/spline_cbfs/blob/7538ee0b9ca8082b2aa1eeda8b5982a17ac5bee0/simulations/simulation1/simulation.py#L14). Note that you should determine (by adding a new column) all other vehicles that will be seen as obstacles to the new robot;
  - Create a new EllipticalBarrier object at line [16](https://github.com/gandradeng/spline_cbfs/blob/7538ee0b9ca8082b2aa1eeda8b5982a17ac5bee0/simulations/simulation1/simulation.py#L16) and add it to the list of barriers at the creation of the BarrierGrid object at line [17](https://github.com/gandradeng/spline_cbfs/blob/7538ee0b9ca8082b2aa1eeda8b5982a17ac5bee0/simulations/simulation1/simulation.py#L17);
  - Crete another *controller_parameterN* dictionary, after line [51](https://github.com/gandradeng/spline_cbfs/blob/7538ee0b9ca8082b2aa1eeda8b5982a17ac5bee0/simulations/simulation1/simulation.py#L51), setting the previous connectivity element on the parameters;
  - Crete another *controllerN* object, after line [54](https://github.com/gandradeng/spline_cbfs/blob/7538ee0b9ca8082b2aa1eeda8b5982a17ac5bee0/simulations/simulation1/simulation.py#L54), setting the previous *controller_parametersN* created;
  - Add the created controller object to the list of controllers in line [56](https://github.com/gandradeng/spline_cbfs/blob/7538ee0b9ca8082b2aa1eeda8b5982a17ac5bee0/simulations/simulation1/simulation.py#L56C30-L56C41);
  - **Enjoy!**
  
## Citing the paper: 
```
@InProceedings{Reis2023,
    author = {Matheus F. Reis and Gustavo A. Andrade
              and A. Pedro Aguiar},
    title = {Safe Autonomous Multi-Vehicle Navigation using Path Following
             Control and Spline-based Barrier Functions},
    booktitle = {Proceedings on Sixth Iberian Robotics Conference - Coimbra, Portugal, 2023}
}
```
