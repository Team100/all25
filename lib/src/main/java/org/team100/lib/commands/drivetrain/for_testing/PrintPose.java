package org.team100.lib.commands.drivetrain.for_testing;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Print the current pose to the console.
 */
public class PrintPose extends Command {

    private final SwerveDriveSubsystem m_drive;

    public PrintPose(SwerveDriveSubsystem swerve) {
        m_drive = swerve;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Util.println("Pose: " + m_drive.getPose());
    }
}
