package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.DriveUtil;

/**
 * Transform manual input into a field-relative velocity.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds implements FieldRelativeDriver {
    private final SwerveKinodynamics m_swerveKinodynamics;
    // LOGGERS
    private final FieldRelativeVelocityLogger m_log_twist;

    public ManualFieldRelativeSpeeds(LoggerFactory parent, SwerveKinodynamics swerveKinodynamics) {
        LoggerFactory child = parent.child(this);
        m_log_twist = child.fieldRelativeVelocityLogger(Level.TRACE, "twist");
        m_swerveKinodynamics = swerveKinodynamics;
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     */
    @Override
    public FieldRelativeVelocity apply(SwerveModel state, DriverControl.Velocity input) {
        // clip the input to the unit circle
        final DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);

        // scale to max in both translation and rotation
        // and desaturate to feasibility
        final FieldRelativeVelocity twistM_S = m_swerveKinodynamics.analyticDesaturation(
                DriveUtil.scale(
                        clipped,
                        m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                        m_swerveKinodynamics.getMaxAngleSpeedRad_S()));

        m_log_twist.log(() -> twistM_S);
        return twistM_S;
    }

    @Override
    public void reset(SwerveModel p) {
        //
    }
}
