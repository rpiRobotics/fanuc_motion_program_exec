# fanuc_motion_program_exec

`fanuc_motion_program_exec` is a python module to run a squence of motion primitives (i.e. `moveL`, `moveC`, `moveJ`) on a fanuc controller in a simple way. The python module now support controlling a single or dual robots.

## Pre-request

You will need a physical FANUC robot or `RoboGuide`, the FANUC robot simulation. The following robot options are required.

- KAREL (R632)
- Ascii Program Loader (R790)
- PC Interface (R641)

For dual robot arms, the following additional option is required.

- Multi-group

## Installation on Robots

### RoboGuide

The installation of modulel in RoboGuide includes setting up a cell and loading necessary scripts.

- See

### Real Robot

TBA

## Installation of Python Module

The `fanuc_motion_program_exec` module can be install to local Python installation using the following command executed in the repository root directory.

```
pip install . --user
```

Note that the module is only tested with `python3`.

## Usage

