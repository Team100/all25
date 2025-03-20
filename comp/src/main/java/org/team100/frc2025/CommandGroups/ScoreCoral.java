package org.team100.frc2025.CommandGroups;

import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.SelectCommand;

public class ScoreCoral extends SequentialCommandGroup100 {

    public ScoreCoral(
            LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> scoringPositionSupplier,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM) {
        super(logger);
        addCommands(
                new ParallelDeadlineGroup100(
                        logger,
                        new Embark(
                                m_drive,
                                heedRadiusM,
                                SwerveControllerFactory.byIdentity(logger),
                                profile,
                                targetSector,
                                destination,
                                scoringPositionSupplier),
                        new SetElevator(elevator, 8, true),
                        new SetWrist(wrist, 0.4, true)),

                new SelectCommand<>(
                        Map.of(
                                ScoringPosition.L1,
                                new ScoreL1(logger),
                                ScoringPosition.L2,
                                new ScoreL2(logger,
                                        wrist,
                                        elevator,
                                        tunnel,
                                        targetSector,
                                        destination,
                                        scoringPositionSupplier,
                                        controller,
                                        profile,
                                        m_drive,
                                        heedRadiusM),
                                ScoringPosition.L3,
                                new ScoreL3(logger, wrist, elevator),
                                ScoringPosition.L4,
                                new ScoreL4(logger, wrist, elevator)),
                        scoringPositionSupplier)

        // new Embark(m_drive, SwerveControllerFactory.byIdentity(logger), profile,
        // targetSector, destination, scoringPositionSupplier)

        );

        // L4
        // addCommands(
        // new SetWrist(wrist, 0.5, false),
        // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 45, false),
        // new
        // SetWrist(wrist, 0.5, true)), //45
        // new ParallelDeadlineGroup100(logger, new SetWrist(wrist, 1.25, false), new
        // SetElevatorPerpetually(elevator, 45)),
        // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 35, false),
        // new
        // SetWrist(wrist, 1.25, true)),
        // new ParallelDeadlineGroup100(logger, new WaitCommand(10), new
        // SetWristDutyCycle(wrist,
        // -1), new SetElevatorPerpetually(elevator, 35))
        // // new ParallelDeadlineGroup100(logger, new SetWrist(wrist, 0.5, false), new
        // SetElevatorPerpetually(elevator, 35)),
        // // new SetElevator(elevator, 2, false)

        // // new SetWrist(wrist, 0.5, false),

        // // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 10, false),
        // new
        // SetWrist(wrist, 1.26, false))
        // // new SetWrist(wrist, 0.5, false)

        // );

        // addCommands(
        // new SetWrist(wrist, 0.5, false),
        // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 23, false),
        // new
        // SetWrist(wrist, 0.5, true)), //45
        // new ParallelDeadlineGroup100(logger, new SetWrist(wrist, 1.25, false), new
        // SetElevatorPerpetually(elevator, 23)),
        // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 12, false),
        // new
        // SetWrist(wrist, 1.25, true)),
        // new ParallelDeadlineGroup100(logger, new SetWrist(wrist, 0.5, false), new
        // SetElevatorPerpetually(elevator, 12)),
        // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 2, false), new
        // SetWrist(wrist, 0.5, true))
        // // new ParallelDeadlineGroup100(logger, new WaitCommand(10), new
        // SetWristDutyCycle(wrist, -1), new SetElevatorPerpetually(elevator, 35))

        // );

        // addCommands(
        // new SetWrist(wrist, 0.5, false),
        // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 11, false),
        // new
        // SetWrist(wrist, 0.5, true)), //45
        // new ParallelDeadlineGroup100(logger, new SetWrist(wrist, 1.25, false), new
        // SetElevatorPerpetually(elevator, 11)),
        // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 3.6, false),
        // new
        // SetWrist(wrist, 1.25, true))
        // // new ParallelDeadlineGroup100(logger, new SetWrist(wrist, 0.5, false), new
        // SetElevatorPerpetually(elevator, 6)),
        // // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 2, false),
        // new
        // SetWrist(wrist, 0.5, true))
        // // new ParallelDeadlineGroup100(logger, new WaitCommand(10), new
        // SetWristDutyCycle(wrist, -1), new SetElevatorPerpetually(elevator, 35))

        // );

        // addCommands(
        // // new SetWrist(wrist, 2.8, false),
        // new SetWrist(wrist, 0.5, false),
        // new SetElevator(elevator, 9.5, false),
        // new ParallelDeadlineGroup100(logger, new SetWrist(wrist, 2.387268, false),
        // new
        // SetElevatorPerpetually(elevator, 9)),
        // new ParallelDeadlineGroup100(logger, new RunCoralTunnel(tunnel, -1), new
        // SetElevatorPerpetually(elevator, 8.1))
        // // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 3.6,
        // false), new
        // SetWrist(wrist, 1.25, true))
        // // new ParallelDeadlineGroup100(logger, new SetWrist(wrist, 0.5, false), new
        // SetElevatorPerpetually(elevator, 6)),
        // // new ParallelDeadlineGroup100(logger, new SetElevator(elevator, 2, false),
        // new
        // SetWrist(wrist, 0.5, true))
        // // new ParallelDeadlineGroup100(logger, new WaitCommand(10), new
        // SetWristDutyCycle(wrist, -1), new SetElevatorPerpetually(elevator, 35))

        // );
    }
}
