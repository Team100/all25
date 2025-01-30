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

    private final SwerveDriveSubsystem m_robotDrive;
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
    Control100 refTheta;

    private boolean m_steeringAligned;

    public Rotate(
            LoggerFactory parent,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            Gyro gyro,
            SwerveKinodynamics swerveKinodynamics,
            double targetAngleRadians) {
        LoggerFactory child = parent.child(this);
        m_robotDrive = drivetrain;
        m_controller = controller;
        m_gyro = gyro;
        m_swerveKinodynamics = swerveKinodynamics;
        m_goalState = new Model100(targetAngleRadians, 0);
        refTheta = new Control100(0, 0);

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
        ChassisSpeeds initialSpeeds = m_robotDrive.getChassisSpeeds();
        refTheta = new Control100(
                m_robotDrive.getPose().getRotation().getRadians(),
                initialSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        // reference
        refTheta = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, refTheta.model(), m_goalState);
        m_finished = MathUtil.isNear(refTheta.x(), m_goalState.x(), kXToleranceRad)
                && MathUtil.isNear(refTheta.v(), m_goalState.v(), kVToleranceRad_S);

        SwerveModel measurement = m_robotDrive.getState();
        Pose2d currentPose = measurement.pose();

        SwerveModel reference = new SwerveModel(
                new Model100(currentPose.getX(), 0), // stationary at current pose
                new Model100(currentPose.getY(), 0),
                new Model100(refTheta.x(), refTheta.v()));

        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(measurement, reference);

        if (m_steeringAligned) {
            // steer normally.
            // there's no feasibility issue because cartesian speed is zero.
            m_robotDrive.driveInFieldCoords(fieldRelativeTarget);
        } else {
            boolean aligned = m_robotDrive.steerAtRest(fieldRelativeTarget);
            // while waiting for the wheels, hold the profile at the start.
            resetRefTheta();
            if (aligned) {
                m_steeringAligned = true;
            }
        }

        double headingMeasurement = currentPose.getRotation().getRadians();
        double headingRate = m_gyro.getYawRateNWU();

        // log what we did
        m_log_error_x.log(() -> refTheta.x() - headingMeasurement);
        m_log_error_v.log(() -> refTheta.v() - headingRate);
        m_log_measurement_x.log(() -> headingMeasurement);
        m_log_measurement_v.log(() -> headingRate);
        m_log_reference.log(() -> refTheta);
    }

    @Override
    public boolean isFinished() {
        return m_finished && m_controller.atReference();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.stop();
    }
}
