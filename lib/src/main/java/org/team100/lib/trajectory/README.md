# trajectory

Team 100 trajectories are adapted from 254.  Trajectories are holonomic,
i.e. handle course and heading separately: "point" below means four-dimensional holonomic state.

The process of constructing a trajectory has three stages:

1. Construct a list of splines.  Each spline joins two points smoothly.  The spline parameter has no physical meaning, it's just 0.0 on one end and 1.0 on the other.  See the "spline" package.

2. Construct a list of points along the spline, such that straight lines connecting the points don't deviate too much from the true spline. (This uses recursive bisection.)  These points will be close together where the curvature is high, and far apart along straighter sections of the spline.  The list is created by PathFactory, producing Path100, which integrates along the list to find the distance.  See the "path" package.

3. Construct a list of points interpolated along the path, such that the points aren't too far apart.  Using a set of kinodynamic constraints (see "timing"), assign a time for each point.  The resulting list of timed poses is created by ScheduleGenerator, producing Trajectory100.  See the "trajectory" package.