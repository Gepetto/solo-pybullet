# solo-pybullet
**Simulation and Controller code for Solo Quadruped**

This repository offers an environment to simulate different controllers on the Quadruped robot **Solo**.

You can implement your controller on the *controller.py* file and call your control function in the main program *main.py* by replacing the `c(...)` function in the loop.

## Installation
To install Pinocchio : follow the instructions on https://stack-of-tasks.github.io/pinocchio/download.html

To download the urdf and meshes of the **Solo** Quadruped:
```
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub xenial robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
sudo apt update -qqy && sudo apt install -qqy robotpkg-example-robot-data
```

To install PyBullet:
`pip install --user pybullet`

## How to start the simulation
Run main.py
