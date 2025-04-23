package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.CommandGroups.DeadlineForEmbarkAndPrePlace;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.HoldWristAndElevator;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.Command;


public class ScoreL4Smart extends SequentialCommandGroup100 {
    public  ScoreL4Smart(LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> height,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            ReefPoint reefPoint) {
        super(logger, "ScoreL4Smart");

        Command holdingCommand = new HoldWristAndElevator(elevator, wrist);

        Embark embarkCommand = new Embark(m_logger, m_drive, heedRadiusM, controller, profile, targetSector, destination, height, reefPoint, true);
        PrePlaceCoralL4 prePlaceCoralL4 = new PrePlaceCoralL4(wrist, elevator, tunnel, 47, true);

        addCommands(
                new ParallelDeadlineGroup100(m_logger, "drive",
                        new DeadlineForEmbarkAndPrePlace(embarkCommand::isDone, prePlaceCoralL4::isDone),
                        embarkCommand,
                        new SequentialCommandGroup100(m_logger, "out",
                                new SetWrist(wrist, 0.4, false),
                                prePlaceCoralL4)),

                // new SetElevatorAndWrist(elevator, wrist, 47, 1.25),
                new PostDropCoralL4(wrist, elevator, 10, holdingCommand)


        );
    }
}
