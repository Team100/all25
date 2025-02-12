package org.team100.lib.reference;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.util.Takt;

/** Produces references based on a trajectory. */
public class TrajectoryReference implements SwerveReference {
    private final Trajectory100 m_trajectory;
    private double m_startTimeS;

    public TrajectoryReference(Trajectory100 trajectory) {
        m_trajectory = trajectory;
    }

    /** Ignores the measurement, resets the trajectory timer. */
    @Override
    public void initialize(SwerveModel measurement) {
        m_startTimeS = Takt.get();
    }

    @Override
    public SwerveModel current() {
        return sample(progress());
    }

    @Override
    public SwerveModel next() {
        return sample(progress() + TimedRobot100.LOOP_PERIOD_S);
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

    private SwerveModel sample(double t) {
        return SwerveModel.fromTimedPose(m_trajectory.sample(t));
    }

}
