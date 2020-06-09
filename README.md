# solo-pybullet

[![Pipeline status](https://gepgitlab.laas.fr/gepetto/solo-pybullet/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/gepetto/solo-pybullet/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/gepetto/solo-pybullet/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/gepetto/solo-pybullet/master/coverage/)

**Simulation and Controller code for Solo Quadruped**

This repository offers an environment to simulate different controllers on the Quadruped robot **Solo**.

You can implement your controller on the *controller.py* file and call your control function in the main program *main.py* by replacing the `c(...)` function in the loop.

## Installation

To install [Pinocchio](https://github.com/stack-of-tasks/pinocchio/), the urdf and meshes of the **Solo** Quadruped,
the [Gepetto Viewer](https://github.com/gepetto/gepetto-viewer-corba) and their python bindings:

```bash
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -cs) robotpkg
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg
EOF
sudo apt update -qqy && sudo apt install -qqy robotpkg-py3\*-{pinocchio,example-robot-data,qt4-gepetto-viewer-corba}
```

To install PyBullet:
`pip3 install --user pybullet`

## How to start the simulation
launch `gepetto-gui`, then `python3 -m solo_pybullet`
