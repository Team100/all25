package org.team100.lib.motion.drivetrain.kinodynamics;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.Gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Kinematics and dynamics of the swerve drive.
 * 
 * Includes speed limits, dynamic constraints, and kinematics.
 * 
 * This class represents *absolute maxima.*
 * 
 * Do not use this class to configure driver preferences, use a command or
 * control instead.
 * 
 * In particular, the maximum spin rate is likely to seem quite high. Do not
 * lower it here.
 */
public class SwerveKinodynamics implements Glassy {
    // geometry
    private final double m_fronttrack;
    private final double m_backtrack;
    private final double m_wheelbase;
    private final double m_frontoffset;
    private final double m_radius;
    private final double m_vcg;
    private final SwerveDriveKinematics100 m_kinematics;
    private final double m_maxCapsizeAccelM_S2;

    // configured inputs
    private final double m_maxDriveVelocityM_S;
    private final double m_stallAccelerationM_S2;
    private final double m_maxDriveAccelerationM_S2;
    private final double m_maxDriveDecelerationM_S2;
    private final double m_maxSteeringVelocityRad_S;

    // calculated
    private final double m_maxAngleSpeedRad_S;
    private final double m_maxAngleAccelRad_S2;
    private final Profile100 m_steeringProfile;

    /**
     * @param maxDriveVelocity        module drive speed m/s
     * @param stallAcceleration       acceleration at stall, used to compute
     *                                back-EMF-limited acceleration at higher RPMs
     * @param maxDriveAcceleration    module drive accel m/s^2
     * @param maxDriveDeceleration    module drive decel m/s^2. Should be higher
     *                                than accel limit, this is a positive number.
     * @param maxSteeringVelocity     module steering axis rate rad/s
     * @param maxSteeringAcceleration module steering axis accel rad/s^2
     * @param fronttrack              meters
     * @param backtrack               meters
     * @param wheelbase               meters
     * @param frontoffset             distance from the center of mass to the front
     *                                wheels, meters
     * @param vcg                     vertical center of gravity, meters
     */
    SwerveKinodynamics(
            double maxDriveVelocity,
            double stallAcceleration,
            double maxDriveAcceleration,
            double maxDriveDeceleration,
            double maxSteeringVelocity,
            double maxSteeringAcceleration,
            double fronttrack,
            double backtrack,
            double wheelbase,
            double frontoffset,
            double vcg) {
        if (fronttrack < 0.1 || backtrack < 0.1)
            throw new IllegalArgumentException();
        if (wheelbase < 0.1)
            throw new IllegalArgumentException();

        m_fronttrack = fronttrack;
        m_backtrack = backtrack;
        m_wheelbase = wheelbase;
        m_frontoffset = frontoffset;
        m_vcg = vcg;
        // distance from center to wheel
        m_radius = Math.hypot((fronttrack + backtrack) / 4, m_wheelbase / 2);
        m_kinematics = new SwerveDriveKinematics100(
                new Translation2d(m_frontoffset, m_fronttrack / 2),
                new Translation2d(m_frontoffset, -m_fronttrack / 2),
                new Translation2d(m_frontoffset - m_wheelbase, m_backtrack / 2),
                new Translation2d(m_frontoffset - m_wheelbase, -m_backtrack / 2));

        // fulcrum is the distance from the center to the nearest edge.
        double fulcrum = Math.min(Math.min(m_fronttrack, m_backtrack) / 2, m_wheelbase / 2);
        m_maxCapsizeAccelM_S2 = 9.8 * (fulcrum / m_vcg);

        m_maxDriveVelocityM_S = maxDriveVelocity;

        m_stallAccelerationM_S2 = stallAcceleration;
        m_maxDriveAccelerationM_S2 = maxDriveAcceleration;
        m_maxDriveDecelerationM_S2 = maxDriveDeceleration;
        m_maxSteeringVelocityRad_S = maxSteeringVelocity;
        m_maxAngleSpeedRad_S = m_maxDriveVelocityM_S / m_radius;

        double accel = Math.max(m_maxDriveAccelerationM_S2, m_maxDriveDecelerationM_S2);
        m_maxAngleAccelRad_S2 = 12 * accel * m_radius
                / (m_fronttrack * m_backtrack + m_wheelbase * m_wheelbase);

        m_steeringProfile = new TrapezoidProfile100(
                m_maxSteeringVelocityRad_S,
                maxSteeringAcceleration,
                0.02); // one degree
    }

    public Profile100 getSteeringProfile() {
        return m_steeringProfile;
    }

    /** Cruise speed, m/s. */
    public double getMaxDriveVelocityM_S() {
        return m_maxDriveVelocityM_S;
    }

    /**
     * Acceleration at stall, without current limiting. Used to compute
     * back-EMF-limited torque available at higher RPMs.
     */
    public double getStallAccelerationM_S2() {
        return m_stallAccelerationM_S2;
    }

    /** Motor-torque-limited acceleration rate, m/s^2 */
    public double getMaxDriveAccelerationM_S2() {
        return m_maxDriveAccelerationM_S2;
    }

    /**
     * Motor-torque-limited drive deceleration rate, m/s^2. Motors are better at
     * slowing down than speeding up, so this should be larger than the accel rate.
     */
    public double getMaxDriveDecelerationM_S2() {
        return m_maxDriveDecelerationM_S2;
    }

    /** Cruise speed of the swerve steering axes, rad/s. */
    public double getMaxSteeringVelocityRad_S() {
        return m_maxSteeringVelocityRad_S;
    }

    /** Spin cruise speed, rad/s. Computed from drive and frame size. */
    public double getMaxAngleSpeedRad_S() {
        return m_maxAngleSpeedRad_S;
    }

    /**
     * Motor-torque-limited spin accel rate, rad/s^2. Computed from drive and frame
     * size.
     */
    public double getMaxAngleAccelRad_S2() {
        return m_maxAngleAccelRad_S2;
    }

    /**
     * Acceleration which will tip the robot onto two wheels, m/s^2. Computed from
     * vertical center of gravity and frame size.
     */
    public double getMaxCapsizeAccelM_S2() {
        return m_maxCapsizeAccelM_S2;
    }

    /**
     * Inverse kinematics, chassis speeds => module states.
     * 
     * The resulting state speeds are always positive.
     * 
     * This version does **DISCRETIZATION** to correct for swerve veering.
     * 
     * It also does extra veering correction proportional to rotation rate and
     * translational acceleration.
     * 
     * States may include empty angles for motionless wheels.
     */
    public SwerveModuleStates toSwerveModuleStates(ChassisSpeeds in) {
        return toSwerveModuleStates(in, TimedRobot100.LOOP_PERIOD_S);
    }

    /**
     * Discretizes.
     * 
     * If you want desaturation, use the setpoint generator.
     * 
     * States may include empty angles for motionless wheels.
     * Otherwise angle is always within [-pi, pi].
     */
    SwerveModuleStates toSwerveModuleStates(ChassisSpeeds in, double dt) {
        // This is the extra correction angle ...
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(in.omegaRadiansPerSecond));
        // ... which is subtracted here; this isn't really a field-relative
        // transformation it's just a rotation.
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                in.vxMetersPerSecond,
                in.vyMetersPerSecond,
                in.omegaRadiansPerSecond,
                angle);
        // discretization does not affect omega
        DiscreteSpeed descretized = discretize(chassisSpeeds, dt);
        SwerveModuleStates states = m_kinematics.toSwerveModuleStates(descretized);
        return states;
    }

    /**
     * Given a desired instantaneous speed, extrapolate ahead one step, and return
     * the twist required to achieve that state.
     * 
     * Tangent velocity
     */
    public static DiscreteSpeed discretize(ChassisSpeeds chassisSpeeds, double dt) {
        Pose2d desiredDeltaPose = new Pose2d(
                chassisSpeeds.vxMetersPerSecond * dt,
                chassisSpeeds.vyMetersPerSecond * dt,
                new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * dt));

        return new DiscreteSpeed(Pose2d.kZero.log(desiredDeltaPose), dt);

        // return new ChassisSpeeds(twist.dx / period, twist.dy / period, twist.dtheta /
        // period);
    }

    /**
     * Returns the "instantaneous" chassis speeds corresponding to the module
     * states, i.e. the chassis speed pointing at the result of applying the module
     * state twist.
     * 
     * This could be used with odometry, but because odometry uses module positions
     * instead of velocities, it is not needed.
     * 
     * It performs inverse discretization and an extra correction.
     * 
     * TODO: make sure the callers of this function are doing the right thing with
     * the result.
     */
    public ChassisSpeeds toChassisSpeedsWithDiscretization(
            SwerveModuleStates moduleStates,
            double dt) {
        ChassisSpeeds discreteSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
        Twist2d twist = new Twist2d(
                discreteSpeeds.vxMetersPerSecond * dt,
                discreteSpeeds.vyMetersPerSecond * dt,
                discreteSpeeds.omegaRadiansPerSecond * dt);

        Pose2d deltaPose = GeometryUtil.sexp(twist);
        ChassisSpeeds continuousSpeeds = new ChassisSpeeds(
                deltaPose.getX(),
                deltaPose.getY(),
                deltaPose.getRotation().getRadians()).div(dt);

        double omega = discreteSpeeds.omegaRadiansPerSecond;
        // This is the opposite direction
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(omega));
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                continuousSpeeds.vxMetersPerSecond,
                continuousSpeeds.vyMetersPerSecond,
                continuousSpeeds.omegaRadiansPerSecond,
                angle.unaryMinus());
    }

    public SwerveDrivePoseEstimator100 newPoseEstimator(
            LoggerFactory parent,
            Gyro gyro,
            SwerveModulePositions modulePositions,
            Pose2d initialPoseMeters,
            double timestampSeconds) {
        return new SwerveDrivePoseEstimator100(
                parent,
                this,
                gyro,
                modulePositions,
                initialPoseMeters,
                timestampSeconds);
    }

    /**
     * Maintain translation and rotation proportionality but slow to a feasible
     * velocity, assuming the robot has an infinite number of wheels on a circular
     * frame.
     */
    public ChassisSpeeds analyticDesaturation(ChassisSpeeds speeds) {
        double maxV = getMaxDriveVelocityM_S();
        double maxOmega = getMaxAngleSpeedRad_S();
        double xySpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double xyAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
        // this could be negative if xySpeed is too high
        double omegaForSpeed = maxOmega * (1 - xySpeed / maxV);
        if (Math.abs(speeds.omegaRadiansPerSecond) <= omegaForSpeed) {
            // omega + xyspeed is feasible
            return speeds;
        }
        if (xySpeed < 1e-12) {
            // if we got here then omega alone is infeasible so use maxomega
            return new ChassisSpeeds(0, 0, Math.signum(speeds.omegaRadiansPerSecond) * maxOmega);
        }
        if (Math.abs(speeds.omegaRadiansPerSecond) < 1e-12) {
            // if we got here then xyspeed alone is infeasible so use maxV
            return new ChassisSpeeds(maxV * Math.cos(xyAngle), maxV * Math.sin(xyAngle), 0);
        }

        double v = maxOmega * xySpeed * maxV / (maxOmega * xySpeed + Math.abs(speeds.omegaRadiansPerSecond) * maxV);

        double vRatio = v / xySpeed;

        return new ChassisSpeeds(
                vRatio * speeds.vxMetersPerSecond,
                vRatio * speeds.vyMetersPerSecond,
                vRatio * speeds.omegaRadiansPerSecond);
    }

    /**
     * Input could be field-relative or robot-relative, the math doesn't depend on
     * theta, because it treats the robot like a circle.
     * 
     * Input must be full-scale, meters/sec and radians/sec, this won't work on
     * control units [-1,1].
     * 
     * @param speeds twist in m/s and rad/s
     * @return
     */
    public FieldRelativeVelocity analyticDesaturation(FieldRelativeVelocity speeds) {
        double maxV = getMaxDriveVelocityM_S();
        double maxOmega = getMaxAngleSpeedRad_S();
        double xySpeed = speeds.norm();
        double xyAngle = Math.atan2(speeds.y(), speeds.x());
        // this could be negative if xySpeed is too high
        double omegaForSpeed = maxOmega * (1 - xySpeed / maxV);
        if (Math.abs(speeds.theta()) <= omegaForSpeed) {
            // omega + xyspeed is feasible
            return speeds;
        }
        if (xySpeed < 1e-12) {
            // if we got here then omega alone is infeasible so use maxomega
            return new FieldRelativeVelocity(0, 0, Math.signum(speeds.theta()) * maxOmega);
        }
        if (Math.abs(speeds.theta()) < 1e-12) {
            // if we got here then xyspeed alone is infeasible so use maxV
            return new FieldRelativeVelocity(maxV * Math.cos(xyAngle), maxV * Math.sin(xyAngle), 0);
        }

        double v = maxOmega * xySpeed * maxV / (maxOmega * xySpeed + Math.abs(speeds.theta()) * maxV);

        double vRatio = v / xySpeed;

        return new FieldRelativeVelocity(
                vRatio * speeds.x(),
                vRatio * speeds.y(),
                vRatio * speeds.theta());
    }

    /** Scales translation to accommodate the rotation. */
    public ChassisSpeeds preferRotation(ChassisSpeeds speeds) {
        double oRatio = Math.min(1, speeds.omegaRadiansPerSecond / getMaxAngleSpeedRad_S());
        double xySpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double maxV = getMaxDriveVelocityM_S();
        double xyRatio = Math.min(1, xySpeed / maxV);
        double ratio = Math.min(1 - oRatio, xyRatio);
        double xyAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);

        return new ChassisSpeeds(
                ratio * maxV * Math.cos(xyAngle),
                ratio * maxV * Math.sin(xyAngle),
                speeds.omegaRadiansPerSecond);
    }

    /** Scales translation to accommodate the rotation. */
    public FieldRelativeVelocity preferRotation(FieldRelativeVelocity speeds) {
        double oRatio = Math.min(1, speeds.theta() / getMaxAngleSpeedRad_S());
        double xySpeed = speeds.norm();
        double maxV = getMaxDriveVelocityM_S();
        double xyRatio = Math.min(1, xySpeed / maxV);
        double ratio = Math.min(1 - oRatio, xyRatio);
        double xyAngle = Math.atan2(speeds.y(), speeds.x());

        return new FieldRelativeVelocity(
                ratio * maxV * Math.cos(xyAngle),
                ratio * maxV * Math.sin(xyAngle),
                speeds.theta());
    }

    /**
     * Robot-relative speed, without discretization.
     * This simply rotates the velocity from the field frame to the robot frame.
     */
    public static ChassisSpeeds toInstantaneousChassisSpeeds(
            FieldRelativeVelocity v,
            Rotation2d theta) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                v.x(),
                v.y(),
                v.theta(),
                theta);
    }

    /**
     * Field-relative speed, without discretization.
     * This simply rotates the velocity from the robot frame to the field frame.
     */
    public static FieldRelativeVelocity fromInstantaneousChassisSpeeds(ChassisSpeeds instantaneous, Rotation2d theta) {
        ChassisSpeeds c = ChassisSpeeds.fromRobotRelativeSpeeds(instantaneous, theta);
        return new FieldRelativeVelocity(c.vxMetersPerSecond, c.vyMetersPerSecond, c.omegaRadiansPerSecond);
    }

    public SwerveDriveKinematics100 getKinematics() {
        return m_kinematics;
    }

}