package org.team100.lib.commands.drivetrain.manual;

import java.util.Optional;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Transform manual input into swerve module states, in a simpler way than
 * ManualModuleStates does.
 * 
 * The input dtheta is exactly the module angle.
 * The input dx is exactly the wheel speed.
 * The input dy is ignored.
 */
public class SimpleManualModuleStates implements ModuleStateDriver {
    private final SwerveKinodynamics m_swerveKinodynamics;
    // LOGGERS
    private final DoubleLogger m_log_speed;
    private final Rotation2dLogger m_log_angle;

    public SimpleManualModuleStates(LoggerFactory parent, SwerveKinodynamics swerveKinodynamics) {
        LoggerFactory child = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_log_speed = child.doubleLogger(Level.TRACE, "speed m_s");
        m_log_angle = child.rotation2dLogger(Level.TRACE, "angle rad");
    }

    /**
     * There's no conflict between translation and rotation velocities in this mode.
     */
    @Override
    public SwerveModuleStates apply(DriverControl.Velocity input) {
        // dtheta is from [-1, 1], so angle is [-pi, pi]
        Optional<Rotation2d> angle = Optional.of(Rotation2d.fromRadians(Math.PI * input.theta()));
        double speedM_S = m_swerveKinodynamics.getMaxDriveVelocityM_S() * input.x();
        m_log_speed.log( () -> speedM_S);
        m_log_angle.log( angle::get);

        return new SwerveModuleStates (
                new SwerveModuleState100(speedM_S, angle),
                new SwerveModuleState100(speedM_S, angle),
                new SwerveModuleState100(speedM_S, angle),
                new SwerveModuleState100(speedM_S, angle)
        );
    }
}
