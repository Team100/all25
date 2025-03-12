package org.team100.lib.localization;

import java.util.Map;
import java.util.Map.Entry;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleDeltas;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class SwerveDrivePoseEstimator100 implements PoseEstimator100, Glassy {
    private static final boolean DEBUG = false;
    /**
     * The buffer only needs to be long enough to catch stale-but-still-helpful
     * vision updates.
     * 
     * The current Raspberry Pi cameras seem to be able to provide frames to RoboRIO
     * code with about 75-100 ms latency. There will never be a vision update
     * older than about 200 ms.
     */
    static final double kBufferDuration = 0.2;

    private final SwerveKinodynamics m_kinodynamics;
    private final TimeInterpolatableBuffer100<InterpolationRecord> m_poseBuffer;
    // LOGGERS
    private final Rotation2dLogger m_log_offset;
    private final DoubleLogger m_log_pose_x;

    /**
     * maintained in resetPosition().
     */
    private Rotation2d m_gyroOffset;

    /**
     * @param kinodynamics      A correctly-configured kinodynamics object
     *                          for your drivetrain.
     * @param gyroAngle         The current gyro angle.
     * @param modulePositions   The current distance and rotation
     *                          measurements of the swerve modules.
     * @param initialPoseMeters The starting pose estimate.
     */
    public SwerveDrivePoseEstimator100(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            Gyro gyro,
            SwerveModulePositions modulePositions,
            Pose2d initialPoseMeters,
            double timestampSeconds) {
        LoggerFactory child = parent.child(this);
        m_kinodynamics = kinodynamics;
        m_poseBuffer = new TimeInterpolatableBuffer100<>(
                kBufferDuration,
                timestampSeconds,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        new SwerveModel(
                                initialPoseMeters,
                                new FieldRelativeVelocity(0, 0, 0)),
                        modulePositions));
        Rotation2d gyroAngle = gyro.getYawNWU();
        m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
        m_log_offset = child.rotation2dLogger(Level.TRACE, "GYRO OFFSET");
        m_log_pose_x = child.doubleLogger(Level.TRACE, "posex");
    }

    /**
     * Sample the state estimate buffer.
     */
    @Override
    public SwerveModel get(double timestampSeconds) {
        return m_poseBuffer.get(timestampSeconds).m_state;
    }

    /** Empty the buffer and add the given measurements. */
    public void reset(
            Gyro gyro,
            SwerveModulePositions modulePositions,
            Pose2d pose,
            double timestampSeconds) {
        Rotation2d gyroAngle = gyro.getYawNWU();
        m_gyroOffset = pose.getRotation().minus(gyroAngle);

        // empty the buffer and add the current pose
        m_poseBuffer.reset(
                timestampSeconds,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        new SwerveModel(pose, new FieldRelativeVelocity(0, 0, 0)),
                        modulePositions));

        m_log_offset.log(() -> m_gyroOffset);
    }

    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     */
    @Override
    public void put(
            double timestampS,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma) {

        // Skip too-old measurement
        if (m_poseBuffer.tooOld(timestampS)) {
            return;
        }

        // Sample the history at the measurement time.
        InterpolationRecord sample = m_poseBuffer.get(timestampS);

        // Nudge the sample towards the measurement.
        Pose2d nudged = nudge(sample.m_state.pose(), measurement, stateSigma, visionSigma);

        // Add the result
        m_poseBuffer.put(
                timestampS,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        new SwerveModel(nudged, sample.m_state.velocity()),
                        sample.m_wheelPositions));

        // Replay odometry after the sample time.
        // (Note the exclusive tailmap: we don't need to see the entry we just added.)
        for (Map.Entry<Double, InterpolationRecord> entry : m_poseBuffer.tailMap(timestampS, false).entrySet()) {
            double entryTimestampS = entry.getKey();
            InterpolationRecord value = entry.getValue();

            // this is what the gyro must have been given the pose and offset
            Rotation2d entryGyroAngle = value.m_state.pose().getRotation().minus(m_gyroOffset);
            double entryGyroRate = value.m_state.theta().v();
            SwerveModulePositions wheelPositions = value.m_wheelPositions;

            put(entryTimestampS, entryGyroAngle, entryGyroRate, wheelPositions);
        }

    }

    /**
     * Nudge the sample towards the measurement.
     */
    static Pose2d nudge(
            Pose2d sample,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma) {
        // Step 2: Measure the twist between the odometry pose and the vision pose.
        Twist2d twist = sample.log(measurement);
        // Step 4: Discount the twist based on the sigmas relative to each other
        Twist2d scaledTwist = getScaledTwist(stateSigma, visionSigma, twist);
        return sample.exp(scaledTwist);
    }

    static Twist2d getScaledTwist(
            double[] stateSigma,
            double[] visionSigma,
            Twist2d twist) {
        // discount the vision update by this factor.
        final double[] k = getK(stateSigma, visionSigma);
        Twist2d scaledTwist = new Twist2d(
                k[0] * twist.dx,
                k[1] * twist.dy,
                k[2] * twist.dtheta);
        return scaledTwist;
    }

    static double[] getK(double[] stateSigma, double[] visionSigma) {
        return new double[] {
                mix(Math.pow(stateSigma[0], 2), Math.pow(visionSigma[0], 2)),
                mix(Math.pow(stateSigma[1], 2), Math.pow(visionSigma[1], 2)),
                mix(Math.pow(stateSigma[2], 2), Math.pow(visionSigma[2], 2))
        };
    }

    /**
     * Put a new state estimate based on gyro and wheel data. These are expected to
     * be current measurements -- there is no history replay here.
     * 
     * The gyro angle overrides the odometry-derived gyro measurement, and
     * the gyro rate overrides the rate derived from the difference to the previous
     * state.
     */
    public void put(double currentTimeS,
            Gyro gyro,
            SwerveModulePositions wheelPositions) {
        put(currentTimeS, gyro.getYawNWU(), gyro.getYawRateNWU(), wheelPositions);
    }

    /**
     * Sample the history before the specified time, and record a new pose based on
     * the difference in wheel positions between the sample and the specified
     * positions.
     */
    void put(
            double currentTimeS,
            Rotation2d gyroAngle,
            double gyroRateRad_S,
            SwerveModulePositions wheelPositions) {

        // the entry right before this one, the basis for integration.
        Entry<Double, InterpolationRecord> lowerEntry = m_poseBuffer.lowerEntry(
                currentTimeS);

        if (lowerEntry == null) {
            // Util.println("lower entry is null");
            // We're at the beginning. There's nothing to apply the wheel position delta to.
            // This should never happen.
            return;
        }

        double dt = currentTimeS - lowerEntry.getKey();
        InterpolationRecord value = lowerEntry.getValue();
        SwerveModel previousState = value.m_state;
        if (DEBUG)
            Util.printf("previous x %.6f y %.6f\n", previousState.pose().getX(), previousState.pose().getY());

        SwerveModuleDeltas modulePositionDelta = DriveUtil.modulePositionDelta(
                value.m_wheelPositions,
                wheelPositions);
        if (DEBUG)
            Util.printf("modulePositionDelta %s\n", modulePositionDelta);

        Twist2d twist = m_kinodynamics.getKinematics().toTwist2d(modulePositionDelta);
        if (DEBUG)
            Util.printf("twist x %.6f y %.6f theta %.6f\n", twist.dx, twist.dy, twist.dtheta);
        // replace the twist dtheta with one derived from the current pose
        // pose angle based on the gyro (which is more accurate)

        Rotation2d angle = gyroAngle.plus(m_gyroOffset);
        if (DEBUG)
            Util.printf("angle %.6f\n", angle.getRadians());
        twist.dtheta = angle.minus(previousState.pose().getRotation()).getRadians();

        Pose2d newPose = previousState.pose().exp(twist);
        if (DEBUG)
            Util.printf("new pose x %.6f y %.6f\n", newPose.getX(), newPose.getY());
        m_log_pose_x.log(newPose::getX);

        // this is the backward finite difference velocity from odometry
        FieldRelativeDelta odoVelo = FieldRelativeDelta.delta(
                previousState.pose(), newPose)
                .div(dt);

        // use the gyro rate instead of the odometry-derived rate
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(
                odoVelo.getX(),
                odoVelo.getY(),
                gyroRateRad_S);

        SwerveModel swerveState = new SwerveModel(newPose, velocity);

        m_poseBuffer.put(
                currentTimeS,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(), swerveState, wheelPositions));
    }

    int size() {
        return m_poseBuffer.size();
    }

    double lastKey() {
        return m_poseBuffer.lastKey();
    }

    ///////////////////////////////////////

    /**
     * Given q and r stddev's, what mixture should that yield?
     * This is the "closed form Kalman gain for continuous Kalman filter with A = 0
     * and C = I. See wpimath/algorithms.md." ... but really it's just a mixer.
     * 
     * @param q state variance
     * @param r vision variance
     */
    static double mix(final double q, final double r) {
        if (q == 0.0)
            return 0.0;
        return q / (q + Math.sqrt(q * r));
    }
}
