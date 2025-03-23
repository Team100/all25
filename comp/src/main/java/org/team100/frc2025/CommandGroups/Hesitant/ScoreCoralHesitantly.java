package org.team100.frc2025.CommandGroups.Hesitant;

import java.util.Map;
import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.manual.DriveAdjustCoral;
import org.team100.lib.commands.drivetrain.manual.FieldRelativeDriver;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.SelectCommand;

public class ScoreCoralHesitantly extends SequentialCommandGroup100 {
    public ScoreCoralHesitantly(
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
            Supplier<DriverControl.Velocity> twistSupplier,
            Supplier<Boolean> readyToPlace,
            FieldRelativeDriver driver) {
        super(logger);

        addCommands(
                new SetElevatorTargetPosition(elevator, scoringPositionSupplier),
                new EmbarkHesitantly(m_drive, controller, profile, targetSector, destination, elevator),
                new ParallelDeadlineGroup100(logger.child("adjust"),
                        new DriveAdjustCoral(twistSupplier, m_drive, readyToPlace, driver),
                        new SelectCommand<>(
                                Map.of(ScoringPosition.L4, new PreScoreL4Hesitant(logger, wrist, elevator)),
                                elevator::getScoringPosition)),
                new ScoreL4Hesitantly(logger, wrist, elevator)

        );
    }
}
