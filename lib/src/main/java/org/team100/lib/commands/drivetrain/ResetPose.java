package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Set the drivetrain pose to the specified pose.
 * 
 * This is useful to reset where "zero" is.
 * 
 * If you just want to reset rotation, use SetRotation.
 */
public class ResetPose extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final Pose2d m_pose;

    public ResetPose(SwerveDriveSubsystem drive, Pose2d pose) {
        m_drive = drive;
        m_pose = pose;
    }

    @Override
    public void initialize() {
        m_drive.resetPose(m_pose);
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}