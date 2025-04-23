package org.team100.frc2025.Swerve;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForwardSlowly extends Command {
    SwerveDriveSubsystem m_drive;

    public DriveForwardSlowly(SwerveDriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(-0.1, 0, 0);
        m_drive.setChassisSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
