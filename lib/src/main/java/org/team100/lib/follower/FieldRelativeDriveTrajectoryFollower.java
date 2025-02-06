package org.team100.lib.follower;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

/**
 * This is a version of the DriveTrajectoryFollower idea, but using
 * field-relative control instead of robot-relative control.
 */
public interface FieldRelativeDriveTrajectoryFollower extends Glassy {

    void setTrajectory(TrajectoryTimeIterator trajectory);

    /**
     * Makes no attempt to enforce feasibility.
     * 
     * @param timestamp   in seconds, use Takt.get(). TODO: remove this
     * @param measurement measured state
     * @return velocity control
     */
    FieldRelativeVelocity update(double timestamp, SwerveModel measurement);

    /**
     * Note that even though the follower is done, the controller might not be.
     */
    boolean isDone();

}
