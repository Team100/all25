package org.team100.lib.follower;

import java.util.Optional;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

public class FieldRelativeDrivePIDFFollower implements FieldRelativeDriveTrajectoryFollower {

    private static final double kPCartV = 1.0;
    private static final double kPThetaV = 1.0;

    private final DriveTrajectoryFollowerUtil m_util;
    private final boolean m_feedforwardOnly;
    private final double m_kPCart;
    private final double m_kPTheta;

    private TrajectoryTimeIterator m_iter;
    private double m_prevTimeS;

    FieldRelativeDrivePIDFFollower(
            DriveTrajectoryFollowerUtil util,
            boolean feedforwardOnly,
            double kPCart,
            double kPTheta) {
        m_util = util;
        m_feedforwardOnly = feedforwardOnly;
        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
    }

    @Override
    public void setTrajectory(TrajectoryTimeIterator iter) {
        m_iter = iter;
        m_prevTimeS = Double.POSITIVE_INFINITY;
    }

    @Override
    public FieldRelativeVelocity update(double timeS, SwerveModel measurement) {
        if (m_iter == null)
            return FieldRelativeVelocity.zero();

        Optional<TimedPose> optionalSetpoint = getSetpoint(timeS);
        if (!optionalSetpoint.isPresent()) {
            return FieldRelativeVelocity.zero();
        }

        TimedPose setpoint = optionalSetpoint.get();

        FieldRelativeVelocity u_FF = m_util.fieldRelativeFeedforward(setpoint);
        if (m_feedforwardOnly)
            return u_FF;

        FieldRelativeVelocity u_FB;
        if (Experiments.instance.enabled(Experiment.FullStateTrajectoryFollower)) {
            u_FB = m_util.fieldRelativeFullFeedback(
                    measurement,
                    setpoint,
                    m_kPCart,
                    m_kPTheta,
                    kPCartV,
                    kPThetaV);
        } else {
            u_FB = m_util.fieldRelativeFeedback(
                    measurement,
                    setpoint,
                    m_kPCart,
                    m_kPTheta);
        }

        return u_FF.plus(u_FB);
    }

    @Override
    public boolean isDone() {
        return m_iter != null && m_iter.isDone();
    }

    Optional<TimedPose> getSetpoint(double timestamp) {
        double mDt = dt(timestamp);
        Optional<TrajectorySamplePoint> sample_point = m_iter.advance(mDt);
        if (!sample_point.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(sample_point.get().state());
    }

    double dt(double timestamp) {
        if (!Double.isFinite(m_prevTimeS))
            m_prevTimeS = timestamp;
        double mDt = timestamp - m_prevTimeS;
        m_prevTimeS = timestamp;
        return mDt;
    }

}
