# drivetrain

Many drivetrain commands you can use in your comp code.

Commands we used in 2025:

* `DriveToPoseWithProfile`: Drives from the current state (works for starting-in-motion) to the supplied goal and stops.  If you start at rest, the path is roughly a straight line.  This is a good command for general navigation if straight lines are ok.
* `DriveWithTrajectoryFunction`: Drives from the current pose (at rest) using a trajectory calculated on the fly, using a function.  This is good if straight lines will not work, i.e. you need to go around something.
* `SetRotation` and `ResetPose`: used to set, or fix, gyro offset and global pose.