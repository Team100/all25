package org.team100.frc2025.Swerve.Auto;

import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.OuttakeAlgaeGrip;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GoToProcessor extends SequentialCommandGroup100 {
    public GoToProcessor(
            LoggerFactory parent,
            FieldLogger.Log fieldLog,
            SwerveDriveSubsystem m_drive,
            SwerveController holonomicController,
            HolonomicProfile profile,
            AlgaeGrip algaeGrip) {
        super(parent, "GoToProcessor");
        addCommands(
                new DriveToPoseWithProfile(fieldLog,
                        () -> new SwerveModel(new Pose2d(6.187, 0.626, new Rotation2d(3 * Math.PI / 2))),
                        m_drive, holonomicController, profile),
                new OuttakeAlgaeGrip(algaeGrip));
    }
}
