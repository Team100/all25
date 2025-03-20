package org.team100.frc2025.CommandGroups;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
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

public class ScoreL2 extends SequentialCommandGroup100 {
    public ScoreL2(
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
                new SetWrist(wrist, 0.4, false),
                new ParallelDeadlineGroup100(logger,
                        new SetElevator(elevator, 10.5, false),
                        new SetWrist(wrist, 0.4, true)),
                new ParallelDeadlineGroup100(logger,
                        new SetWrist(wrist, 0.9, false),
                        new SetElevatorPerpetually(elevator, 10.5)),
                new ParallelDeadlineGroup100(logger,
                        new SetWrist(wrist, 0.9, false),
                        new SetElevatorPerpetually(elevator, 4.6)),
                new ParallelDeadlineGroup100(logger,
                        new Embark(m_drive, heedRadiusM,
                                SwerveControllerFactory.byIdentity(logger),
                                profile, targetSector,
                                destination, scoringPositionSupplier, 2),
                        new SetWrist(wrist, 1.2, true), new SetElevatorPerpetually(elevator, 4.6))
        // new ParallelDeadlineGroup100(logger,
        // new SetElevator(elevator, 6, false), new
        // SetWrist(wrist, 0.9, true))
        );
    }
}
