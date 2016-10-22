# Physical Simulation

A Python implementation of the "Projective Dynamics: Fusing Constraint Projections for Fast Simulation" paper by Sofien Bouaziz, Sebastian Martin, Tiantian Liu, Ladislav Kavan, Mark Pauly

## Dependencies

Python 3, NumPy, PyGlet, PIL

## To Run

`python3 renderer.py [size] [flag_types]`

`size` is an integer for the size of the flag (i.e. `20` will give a 20 by 20 vert flag). `flag_types` is a space separated argument where each one is either `spring`, `exp_tri`, or `imp_tri`, depending on the type of flags desired (Explicit Spring based, Explicit Triangle elements, or Implicit Triangle Elements).

## Controls

`w` and `s` control the zoom. `r`, `t` control the wind magnitude, which is printed to console. `y` flips the wind magnitude. Mouse controls the rotation around the Y-axis. `f` toggles the triangle borders. `esc` exits the program.
