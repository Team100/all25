# lib.trajectory

This package represents an path in SE(2) and a schedule.

`Trajectory100` is adapted from 254.  Trajectories are holonomic,
i.e. handle course and heading separately, so they have four dimensions (x, y, heading, course), and "point" below means four-dimensional holonomic state.

A good entry point to experiment with trajectories is `TrajectoryPlanner`.  Try `restToRest()` between two poses.

The process of constructing a trajectory has three stages.

1. First, make a list of `WaypointSE2`.  These describe points in the SE(2) manifold that
you want to travel through.  A waypoint includes

   * a _translation_ in the (x,y) plane
   * a _heading_ describing the orientation of the robot, i.e. where the front is facing.
   * a _direction_ of motion.  Note that "direction" in SE(2) has three dimensions.
   * a _scale_ used for the next step.

2. Construct a list of `HolonomicSpline`, with the waypoints above (called "knots") between them.
The "scale" parameter above determines the "straightness" of the curve at each knot.
In our implementation, the curvature at each knot is zero.

3. Construct a list of points (`Pose2dWithMotion`) along the splines, such that straight lines
connecting the points ("secant lines") don't deviate too much from the true spline, and aren't
too far apart from each other. (This uses recursive bisection.)  These points will
be close together where the curvature is high, and far apart along straighter sections
of the spline.  Each point is a `WaypointSE2` and also the spatial rate of heading and course
along the path.  These steps are performed by `PathFactory`, producing `Path100`.

4. Using a list of kinodynamic constraints (implementations of `TimingConstraint`),
assign a time for each point.  The resulting list of `TimedState` is created by `TrajectoryFactory`, producing `Trajectory100`.

To use a trajectory, you `sample()` it, with time (in seconds) as the parameter.  The resulting `TimedState` is interpolated from the list of `TimedState` above.

If you want to use these trajectories for non-holonomic (e.g. "tank") drivetrains, it will work well enough to set the course and heading to be the same at each waypoint.