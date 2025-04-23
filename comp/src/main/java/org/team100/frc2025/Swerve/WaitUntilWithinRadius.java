package org.team100.frc2025.Swerve;

import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitUntilWithinRadius extends Command {
    SwerveDriveSubsystem m_drive;
    boolean finished = false;

    public WaitUntilWithinRadius(SwerveDriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        if (FieldConstants.getDistanceToReefCenter(m_drive.getPose().getTranslation()) < 2) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
