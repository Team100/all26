# lib.state

This package represents mechanisms:

- `StateR1` represents measurement in one dimension. Measurements never include acceleration,
since it is not directly measurable.
- `ControlR1` represents control outputs in one dimension, which _do_ contain acceleration,
which can translate directly into motor voltages using the "kA" factor of the motor models.

In the "state space" representation in control theory, the `StateR1` is
the `x` variable and `ControlR1` is the `u` variable.

There's also a 2d version, `StateR2` and `ControlR2`, for planar positional mechanisms.

There are groupings for the SE(2) manifold of 2d transformations, `StateSE2`
and `ControlSE2`.  These treat each dimension independently, not using the
logmap geodesic constant-twist idea.