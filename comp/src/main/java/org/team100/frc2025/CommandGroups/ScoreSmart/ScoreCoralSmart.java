package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.FieldConstants.ReefPoint;
import org.team100.frc2025.CommandGroups.ScoreL2;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.SelectCommand;

public class ScoreCoralSmart extends SequentialCommandGroup100 {
    public ScoreCoralSmart(LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> scoringPositionSupplier,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            ReefPoint point) {
        super(logger, "ScoreCoralSmart");
        addCommands(
                new SelectCommand<>(
                        Map.of(
                            ScoringPosition.L4, new ScoreL4Smart(m_logger, wrist, elevator, tunnel, targetSector, destination, scoringPositionSupplier, controller, profile, m_drive, heedRadiusM, point),
                            ScoringPosition.L3, new ScoreL3Smart(m_logger, wrist, elevator, tunnel, targetSector, destination, scoringPositionSupplier, controller, profile, m_drive, heedRadiusM, point),
                            ScoringPosition.L2, new ScoreL2Smart(m_logger, wrist, elevator, tunnel, targetSector, destination, scoringPositionSupplier, controller, profile, m_drive, heedRadiusM, point)
                        ),
                        scoringPositionSupplier));
    }
}
