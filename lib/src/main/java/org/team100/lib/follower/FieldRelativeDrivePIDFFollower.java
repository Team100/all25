package org.team100.lib.follower;

import java.util.Optional;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.logging.LoggerFactory.TimedPoseLogger;
import org.team100.lib.logging.LoggerFactory.TrajectorySamplePointLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

/**
 * Follows a trajectory using velocity feedforward with optional feedback.
 */
public class FieldRelativeDrivePIDFFollower implements FieldRelativeDriveTrajectoryFollower {

    private static final double kPCartV = 1.0;
    private static final double kPThetaV = 1.0;

    /** Log exists so multiple controllers can use the same keys. */
    public static class Log {
        private final SwerveModelLogger m_log_measurement;
        private final TimedPoseLogger m_log_setpoint;
        private final BooleanLogger m_log_is_mt;
        private final TrajectorySamplePointLogger m_log_sample;

        public Log(LoggerFactory parent) {
            LoggerFactory log = parent.child("FieldRelativeDrivePIDFFollower");
            m_log_measurement = log.swerveModelLogger(Level.DEBUG, "measurement");
            m_log_setpoint = log.timedPoseLogger(Level.DEBUG, "setpoint");
            m_log_is_mt = log.booleanLogger(Level.TRACE, "IS MT");
            m_log_sample = log.trajectorySamplePointLogger(Level.DEBUG, "sample point");
        }
    }

    private final Log m_log;
    private final DriveTrajectoryFollowerUtil m_util;
    private final boolean m_feedforwardOnly;
    private final double m_kPCart;
    private final double m_kPTheta;

    private TrajectoryTimeIterator m_iter;
    private double m_prevTimeS;

    FieldRelativeDrivePIDFFollower(
            Log log,
            DriveTrajectoryFollowerUtil util,
            boolean feedforwardOnly,
            double kPCart,
            double kPTheta) {
        m_log = log;
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

    /**
     * This uses a single setpoint at the current time to compute the error,
     * which is correct, but it also uses that same setpoint for feed forward, which
     * is not correct. TODO: fix it.
     */
    @Override
    public FieldRelativeVelocity update(double timeS, SwerveModel measurement) {
        if (m_iter == null)
            return FieldRelativeVelocity.zero();

        m_log.m_log_measurement.log(() -> measurement);

        Optional<TimedPose> optionalSetpoint = getSetpoint(timeS);
        if (!optionalSetpoint.isPresent()) {
            return FieldRelativeVelocity.zero();
        }

        TimedPose setpoint = optionalSetpoint.get();
        m_log.m_log_setpoint.log(() -> setpoint);

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

    /**
     * this mutates the iterator
     * TODO: fix it.
     */
    Optional<TimedPose> getSetpoint(double timestamp) {
        double mDt = dt(timestamp);
        Optional<TrajectorySamplePoint> sample_point = m_iter.advance(mDt);
        if (!sample_point.isPresent()) {
            m_log.m_log_is_mt.log(() -> true);
            return Optional.empty();
        }
        m_log.m_log_sample.log(sample_point::get);
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
