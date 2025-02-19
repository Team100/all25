package org.team100.lib.commands.drivetrain.for_testing;

import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.util.ParabolicWave;
import org.team100.lib.util.SquareWave;
import org.team100.lib.util.Takt;
import org.team100.lib.util.TriangleWave;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive back and forth forever, for calibration.
 * 
 * This version operates on module states directly.
 * 
 * It uses minimum-time profiles, so the acceleration is a square wave (i.e.
 * infinite jerk), velocity is a triangle wave, so the resulting position should
 * be a piecewise parabolic curve that looks a lot like a sine wave, though it
 * is not one.
 */
public class OscillateDirect extends Command implements Glassy {
    private static final double kAccel = 1;
    private static final double kMaxSpeed = 1;
    private static final double kPeriod = 4 * kMaxSpeed / kAccel;

    private final SwerveDriveSubsystem m_drive;
    private final SquareWave m_square;
    private final TriangleWave m_triangle;
    private final ParabolicWave m_parabola;
    private final Takt.Timer m_timer;

    // LOGGERS
    private final DoubleLogger m_log_period;
    private final DoubleLogger m_log_time;
    private final DoubleLogger m_log_setpoint_accel;
    private final DoubleLogger m_log_setpoint_speed;
    private final DoubleLogger m_log_setpoint_position;
    private final DoubleLogger m_log_measurement_speed;
    private final DoubleLogger m_log_measurement_position;

    SwerveModel m_initial;

    public OscillateDirect(LoggerFactory parent, SwerveDriveSubsystem swerve) {
        LoggerFactory child = parent.child(this);
        m_drive = swerve;
        m_square = new SquareWave(kAccel, kPeriod);
        m_triangle = new TriangleWave(kMaxSpeed, kPeriod);
        m_parabola = new ParabolicWave(kMaxSpeed * kPeriod / 4, kPeriod);
        m_timer = new Takt.Timer();
        addRequirements(m_drive);
        m_log_period = child.doubleLogger(Level.DEBUG, "period");
        m_log_time = child.doubleLogger(Level.DEBUG, "time");
        m_log_setpoint_accel = child.doubleLogger(Level.DEBUG, "setpoint/accel");
        m_log_setpoint_speed = child.doubleLogger(Level.DEBUG, "setpoint/speed");
        m_log_setpoint_position = child.doubleLogger(Level.DEBUG, "setpoint/position");
        m_log_measurement_speed = child.doubleLogger(Level.DEBUG, "measurement/speed");
        m_log_measurement_position = child.doubleLogger(Level.DEBUG, "measurement/position");
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_initial = m_drive.getState();
    }

    @Override
    public void execute() {
        final double time = m_timer.get();
        final double accelM_S_S = m_square.applyAsDouble(time);
        final double speedM_S = m_triangle.applyAsDouble(time);
        final double positionM = m_parabola.applyAsDouble(time);

        // straight ahead
        straight(speedM_S);

        // straight ahead but running motors in reverse
        // note the unit test doesn't like this because the steering is concurrent with
        // the driving, which is a real effect; we could fix it with some sort of "steer
        // at rest" function.
        // back(speedM_S);

        // straight ahead but with the right side inverted, to see what happens.
        // skew(speedM_S);

        // go in a circle by putting the wheels at 45 degrees; the "positive" rolling
        // direction is a negative spin.
        // spin(speedM_S);

        m_log_period.log(() -> kPeriod);
        m_log_time.log(() -> time);
        m_log_setpoint_accel.log(() -> accelM_S_S);
        m_log_setpoint_speed.log(() -> speedM_S);
        m_log_setpoint_position.log(() -> positionM);
        m_log_measurement_speed.log(() -> m_drive.getState().x().v());
        m_log_measurement_position.log(() -> m_drive.getState().x().x() - m_initial.x().x());
    }

    void straight(double speedM_S) {
        m_drive.setRawModuleStates(new SwerveModuleStates (
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)), // FL
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)), // FR
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)), // RL
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)) // RR
        ));
    }

    void sideways(double speedM_S) {
        m_drive.setRawModuleStates(new SwerveModuleStates (
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotation90)), // FL
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotation90)), // FR
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotation90)), // RL
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotation90)) // RR
        ));
    }

    void back(double speedM_S) {
        m_drive.setRawModuleStates(new SwerveModuleStates (
                new SwerveModuleState100(-speedM_S, Optional.of(GeometryUtil.kRotation180)), // FL
                new SwerveModuleState100(-speedM_S, Optional.of(GeometryUtil.kRotation180)), // FR
                new SwerveModuleState100(-speedM_S, Optional.of(GeometryUtil.kRotation180)), // RL
                new SwerveModuleState100(-speedM_S, Optional.of(GeometryUtil.kRotation180)) // RR
        ));
    }

    void skew(double speedM_S) {
        m_drive.setRawModuleStates(new SwerveModuleStates (
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)), // FL
                new SwerveModuleState100(-speedM_S, Optional.of(GeometryUtil.kRotation180)), // FR
                new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)), // RL
                new SwerveModuleState100(-speedM_S, Optional.of(GeometryUtil.kRotation180)) // RR
        ));
    }

    void spin(double speedM_S) {
        m_drive.setRawModuleStates(new SwerveModuleStates (
                new SwerveModuleState100(speedM_S, Optional.of(new Rotation2d(-Math.PI / 4))), // FL
                new SwerveModuleState100(speedM_S, Optional.of(new Rotation2d(-3 * Math.PI / 4))), // FR
                new SwerveModuleState100(speedM_S, Optional.of(new Rotation2d(Math.PI / 4))), // RL
                new SwerveModuleState100(speedM_S, Optional.of(new Rotation2d(3 * Math.PI / 4))) // RR
        ));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
