# Min Curv Planner QP

This project is a Quadratic Programming (QP) based minimum curvature path planner. Below are the instructions to build and execute an example of how to use the library.

## Prerequisites

Ensure you have the following dependencies installed:
- CMake
- OSQP
- OSQP Eigen

### Installing OSQP

```sh
git clone https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake ..
make
sudo make install
```

### Installing OSQP Eigen

```sh
git clone https://github.com/robotology/osqp-eigen
cd osqp-eigen
mkdir build
cd build
cmake ..
make
sudo make install
```

## Building the Project


```sh
git clone https://github.com/yourusername/min_curv_planner_qp.git
cd min_curv_planner_qp
mkdir build
mkdir bin
cd build
cmake ..
make
```

## Executing the Planner Example

After building the project, you can execute the example with:

```sh
./bin/example
```
Ensure you are in the top directory when running the executable.

You can see the program options using the flag ```--help```.
The example uses a sinusoidal wave as initial trajectory and boundaries.

## Visualization
After running the example, the results can be visualized using the [plot_results.py](./scripts/plot_results.py) script:

```sh
./scripts/plot_results.py --path <path to output CSV file>
```

The dependencies are:
- pandas
- matplotlib
- numpy
- Fire
