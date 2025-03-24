package org.team100.frc2025.CommandGroups;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.IntakeAlgaeGrip;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

public class DescoreAlgae extends SequentialCommandGroup100 {

    public DescoreAlgae(
            LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            AlgaeGrip grip,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> height,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM) {
        super(logger, "Descore Algae");

        addCommands(
                new SetAlgaeDescorePositionPrep(m_logger, wrist, elevator),
                // new IntakeAlgaeGrip(grip, true)
                new ParallelDeadlineGroup100(m_logger, "approach",
                        new Embark(m_drive, heedRadiusM, controller, profile, targetSector, destination, height, 1.4),
                        new SetWrist(wrist, 3.7, true),
                        new SetElevatorPerpetually(elevator, 12)),
                new ParallelDeadlineGroup100(m_logger, "elevate",
                        new SetElevator(elevator, 35, false),
                        new SetWrist(wrist, 3.7, true),
                        new IntakeAlgaeGrip(grip, true)),
                new ParallelDeadlineGroup100(m_logger, "pick",
                        new Embark(m_drive, heedRadiusM, controller, profile, targetSector, destination, height, 1.35),
                        new IntakeAlgaeGrip(grip, true),
                        new SetElevatorPerpetually(elevator, 35),
                        new SetWrist(wrist, 3.7, true)),
                new ParallelCommandGroup100(m_logger, "retreat",
                        new Embark(m_drive, heedRadiusM, controller, profile, targetSector, destination, height, 1.75),
                        new IntakeAlgaeGrip(grip, true),
                        new SetElevatorPerpetually(elevator, 35),
                        new SetWrist(wrist, 3.7, true))

        // new SetWrist(wrist, 0.4, false)
        // new ParallelCommandGroup100(
        // new
        // )

        );
    }
}
