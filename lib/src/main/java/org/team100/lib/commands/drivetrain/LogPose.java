package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class LogPose extends Command {

    SwerveDriveSubsystem m_drive;

    public LogPose(SwerveDriveSubsystem swerve) {
        m_drive = swerve;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d currPose = m_drive.getPose();
        Util.println("Pose: " + currPose);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
