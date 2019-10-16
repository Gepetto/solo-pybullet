# solo-pybullet
**Simulation and Controller code for Solo Quadruped**
This repository offers an environment to simulate different controllers on the Quadruped robot **Solo**.
You can implement your controller on the controller.py file and call your control function in the main program main.py by replacing the `c(...)` function in the loop.

## Installation
To install Pinocchio : follow the instructions on https://stack-of-tasks.github.io/pinocchio/download.html

To download the urdf and meshes of the **Solo** Quadruped:
`sudo apt install -qqy robotpkg-example-robot-data`

To install PyBullet:
`pip install pybullet`

## How to start the simulation
Run main.py 
