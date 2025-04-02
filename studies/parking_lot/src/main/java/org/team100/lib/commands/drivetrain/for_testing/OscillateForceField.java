package org.team100.lib.commands.drivetrain.for_testing;

import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Uses a very simple controller to go to a pose.
 * 
 * This is like a simpler version of DriveToPose, or like OscillateProfile
 * without the profile.
 * 
 * It's *very* simple but it works surprisingly well.
 */
public class OscillateForceField extends Command implements Glassy {
    private static final double TOLERANCE = 0.01;

    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final double m_offsetM;

    private SwerveModel m_goal;

    public OscillateForceField(
            SwerveDriveSubsystem swerve,
            SwerveController controller,
            double offsetM) {
        m_drive = swerve;
        m_controller = controller;
        m_offsetM = offsetM;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // choose a goal 1m away
        SwerveModel start = m_drive.getState();
        Pose2d startPose = start.pose();

        Pose2d endPose = startPose.plus(new Transform2d(m_offsetM, 0, new Rotation2d()));

        m_goal = new SwerveModel(endPose);
    }

    @Override
    public void execute() {
        SwerveModel measurement = m_drive.getState();
        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(
            measurement, m_goal, m_goal);
        m_drive.driveInFieldCoords(fieldRelativeTarget);
    }

    @Override
    public boolean isFinished() {
        SwerveModel measurement = m_drive.getState();
        return measurement.near(m_goal, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
