package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.CommandGroups.DeadlineForEmbarkAndPrePlace;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL3;
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

public class ScoreL3Smart extends SequentialCommandGroup100 {
    public ScoreL3Smart(LoggerFactory logger,
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

        Embark embarkCommand = new Embark(m_logger, m_drive, heedRadiusM, controller, profile, targetSector,
                destination, height, reefPoint, true);
        PrePlaceCoralL3 prePlaceCoralL3 = new PrePlaceCoralL3(wrist, elevator, 23, true);

        addCommands(
                new ParallelDeadlineGroup100(m_logger, "drive",
                        new DeadlineForEmbarkAndPrePlace(embarkCommand::isDone, prePlaceCoralL3::isDone),
                        embarkCommand,
                        new SequentialCommandGroup100(m_logger, "out",
                                // new WaitUntilWithinRadius(m_drive),
                                new SetWrist(wrist, 0.4, false),
                                prePlaceCoralL3)),
                // new ParallelDeadlineGroup100(m_logger, "up",
                // new SetWrist(wrist, 0.9, false),
                // new SetElevatorPerpetually(elevator, 23)),
                // new ParallelDeadlineGroup100(m_logger, "score",
                // new SetElevator(m_logger, elevator, 16, false),
                // new SetWrist(wrist, 0.9, true)),
                // // new SetElevator(logger, elevator, 0, isScheduled())
                // new ParallelDeadlineGroup100(m_logger, "down",
                // new SetElevator(m_logger, elevator, 10, false),
                // new SetWrist(wrist, 0.5, true))

                new PostDropCoralL3(wrist, elevator, 10, holdingCommand)

        );
    }
}
