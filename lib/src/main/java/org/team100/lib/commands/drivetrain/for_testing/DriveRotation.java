package org.team100.lib.commands.drivetrain.for_testing;

import java.util.function.Supplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Rotate in place at the specified speed.
 */
public class DriveRotation extends Command implements Glassy {
    private final SwerveDriveSubsystem m_drive;
    private final Supplier<Double> m_omega;

    public DriveRotation(
            SwerveDriveSubsystem drive,
            Supplier<Double> omega) {
        m_drive = drive;
        m_omega = omega;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        double omega = m_omega.get();
        if (Math.abs(omega) <= 0.15) {
            omega = 0;
        }

        m_drive.driveInFieldCoords(new FieldRelativeVelocity(0, 0, omega));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
