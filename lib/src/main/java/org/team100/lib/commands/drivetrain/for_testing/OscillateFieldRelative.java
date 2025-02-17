package org.team100.lib.commands.drivetrain.for_testing;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.ParabolicWave;
import org.team100.lib.util.SquareWave;
import org.team100.lib.util.Takt;
import org.team100.lib.util.TriangleWave;

import edu.wpi.first.wpilibj2.command.Command;

/** Like {@link Oscillate} but field-relative. */
public class OscillateFieldRelative extends Command implements Glassy {
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

    public OscillateFieldRelative(LoggerFactory parent, SwerveDriveSubsystem swerve) {
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

        m_drive.driveInFieldCoords(new FieldRelativeVelocity(speedM_S, 0, 0));

        m_log_period.log(() -> kPeriod);
        m_log_time.log(() -> time);
        m_log_setpoint_accel.log(() -> accelM_S_S);
        m_log_setpoint_speed.log(() -> speedM_S);
        m_log_setpoint_position.log(() -> positionM);
        m_log_measurement_speed.log(() -> m_drive.getState().x().v());
        m_log_measurement_position.log(() -> m_drive.getState().x().x() - m_initial.x().x());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

}
