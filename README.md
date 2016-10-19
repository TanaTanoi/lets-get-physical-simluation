# Physical Simulation

A Python implementation of the "Projective Dynamics: Fusing Constraint Projections for Fast Simulation" paper by Sofien Bouaziz, Sebastian Martin, Tiantian Liu, Ladislav Kavan, Mark Pauly

## To Run

`python3 renderer.py [size] [flag_types]`

`size` is an integer for the size of the flag (i.e. `20` will give a 20 by 20 vert flag). `flag_types` is a space separated argument where each one is either `spring`, `cell`, or `triangle`, depending on the type of flags desired.

## Controls

`w` and `s` control the zoom. `r`, `t` control the wind magnitude, which is printed to console. `y` flips the wind magnitude. Mouse controls the rotation around the Y-axis. `f` toggles the triangle borders. `esc` exits the program.
