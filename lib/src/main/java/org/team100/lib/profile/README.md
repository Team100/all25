# lib.profile

This package supports "profiled" motion, which means movement from
a specified starting state to an ending state, as fast as possible,
within some constraints on velocity and acceleration.

There are several subpackages:

* `timed` profiles are like a trajectories: you precalculate the schedule
and then sample it.  The main interface is `TimedProfile`.

* `incremental` profiles have no state: you give one the current setpoint,
and it produces the next one.  The main interface is `IncrementalProfile`.

* `se2` provides multi-dimensional profiled motion in the SE(2) manifold
(x, y, theta), useful for navigation or planar motion.  The main
interface is `ProfileSE2`.  

* `roadrunner` is a direct translation of the RoadRunner Kotlin classes,
which are used in the `timed` package.



