package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Rotate in place to the specified angle.
 * 
 * Uses a profile with the holonomic drive controller.
 */
public class Rotate extends Command implements Glassy {
    private static final double kXToleranceRad = 0.02;
    private static final double kVToleranceRad_S = 0.02;
    // don't try to rotate at max speed
    private static final double kSpeed = 0.5;

    private final SwerveDriveSubsystem m_drive;
    private final Gyro m_gyro;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Model100 m_goalState;

    // LOGGERS
    private final DoubleLogger m_log_error_x;
    private final DoubleLogger m_log_error_v;
    private final DoubleLogger m_log_measurement_x;
    private final DoubleLogger m_log_measurement_v;
    private final Control100Logger m_log_reference;

    final HolonomicFieldRelativeController m_controller;

    private boolean m_finished = false;

    Profile100 m_profile;
    Control100 m_currentRefTheta;

    private boolean m_steeringAligned;

    public Rotate(
            LoggerFactory parent,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            Gyro gyro,
            SwerveKinodynamics swerveKinodynamics,
            double targetAngleRadians) {
        LoggerFactory child = parent.child(this);
        m_drive = drivetrain;
        m_controller = controller;
        m_gyro = gyro;
        m_swerveKinodynamics = swerveKinodynamics;
        m_goalState = new Model100(targetAngleRadians, 0);
        m_currentRefTheta = new Control100(0, 0);

        addRequirements(drivetrain);
        m_log_error_x = child.doubleLogger(Level.TRACE, "errorX");
        m_log_error_v = child.doubleLogger(Level.TRACE, "errorV");
        m_log_measurement_x = child.doubleLogger(Level.TRACE, "measurementX");
        m_log_measurement_v = child.doubleLogger(Level.TRACE, "measurementV");
        m_log_reference = child.control100Logger(Level.TRACE, "reference");
    }

    @Override
    public void initialize() {
        m_controller.reset();
        resetRefTheta();
        m_profile = new TrapezoidProfile100(
                m_swerveKinodynamics.getMaxAngleSpeedRad_S() * kSpeed,
                m_swerveKinodynamics.getMaxAngleAccelRad_S2() * kSpeed,
                0.05);
        // first align the wheels
        m_steeringAligned = false;
    }

    private void resetRefTheta() {
        ChassisSpeeds initialSpeeds = m_drive.getChassisSpeeds();
        m_currentRefTheta = new Control100(
                m_drive.getPose().getRotation().getRadians(),
                initialSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        if (!m_steeringAligned) {
            // while waiting for the wheels, hold the profile at the start.
            resetRefTheta();
        }

        m_finished = MathUtil.isNear(m_currentRefTheta.x(), m_goalState.x(), kXToleranceRad)
                && MathUtil.isNear(m_currentRefTheta.v(), m_goalState.v(), kVToleranceRad_S);

        SwerveModel measurement = m_drive.getState();
        Pose2d currentPose = measurement.pose();

        SwerveModel currentRef = new SwerveModel(
                new Model100(currentPose.getX(), 0), // stationary at current pose
                new Model100(currentPose.getY(), 0),
                new Model100(m_currentRefTheta.x(), m_currentRefTheta.v()));

        Control100 nextRefTheta = m_profile.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                m_currentRefTheta.model(),
                m_goalState);

        SwerveModel nextReference = new SwerveModel(
                new Model100(currentPose.getX(), 0), // stationary at current pose
                new Model100(currentPose.getY(), 0),
                new Model100(nextRefTheta.x(), nextRefTheta.v()));

        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
                measurement, currentRef, nextReference);

        m_currentRefTheta = nextRefTheta;

        if (!m_steeringAligned && m_drive.aligned(fieldRelativeTarget)) {
            // not aligned before, but are now.
            m_steeringAligned = true;
        }

        if (m_steeringAligned) {
            // steer normally.
            // there's no feasibility issue because cartesian speed is zero.
            m_drive.driveInFieldCoords(fieldRelativeTarget);
        } else {
            m_drive.steerAtRest(fieldRelativeTarget);
        }

        // log what we did

        double headingMeasurement = currentPose.getRotation().getRadians();
        double headingRate = m_gyro.getYawRateNWU();

        m_log_error_x.log(() -> m_currentRefTheta.x() - headingMeasurement);
        m_log_error_v.log(() -> m_currentRefTheta.v() - headingRate);
        m_log_measurement_x.log(() -> headingMeasurement);
        m_log_measurement_v.log(() -> headingRate);
        m_log_reference.log(() -> m_currentRefTheta);
    }

    @Override
    public boolean isFinished() {
        return m_finished && m_controller.atReference();
    }

    @Override
    public void end(boolean isInterupted) {
        m_drive.stop();
    }
}
