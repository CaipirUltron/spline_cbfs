# BSpline-based Path Following using CLF and CBFs

This project consists on the implementation of traffic simulator for autonomous vehicles using a safety-critical path-following controller.
The controller is based on the CLF-CBF framework, and is able to steers the vehicles towards spline-based paths while avoiding collisions.

**Running instructions:** 

$ *python3 simulation.py <example.py in ./scenarios>: **runs simulation**.

$ *python3 animation.py <example.json in ./>: **animates previously saved simulation**.


It is also possible to edit splines with **edit_spline.py**:

$ *python3 edit_spline.py <spline_file.json in ./>: **opens spline editor for a given previously saved spline configuration**.


**Enjoy!**