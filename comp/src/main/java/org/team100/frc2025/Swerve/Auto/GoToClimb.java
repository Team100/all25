package org.team100.frc2025.Swerve.Auto;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.BargeDestination;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GoToClimb extends SequentialCommandGroup{
    public GoToClimb(FieldLogger.Log fieldLog, SwerveDriveSubsystem m_drive, SwerveController holonomicController, HolonomicProfile profile) {

        //TODO make this also deploy climber and climb
        addCommands(new DriveToPoseWithProfile(fieldLog, () -> {
                    Pose2d x = new Pose2d(FieldConstants.getBargeStation(BargeDestination.CENTER, true),
                            new Rotation2d(Math.PI));
                    return new SwerveModel(new Model100(x.getX(), 0
                    ), new Model100(x.getY(), 0),
                            new Model100(x.getRotation().getRadians(), 0));
                }, m_drive, holonomicController, profile), 
                    new DriveToPoseWithProfile(fieldLog, () -> {
                    Pose2d x = new Pose2d(FieldConstants.getBargeStation(BargeDestination.CENTER, false),
                            new Rotation2d(Math.PI));
                    return new SwerveModel(new Model100(x.getX(), 0), new Model100(x.getY(), 0),
                            new Model100(x.getRotation().getRadians(), 0));
                }, m_drive, holonomicController, profile));
    }
}
