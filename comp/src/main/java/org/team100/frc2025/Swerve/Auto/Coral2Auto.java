package org.team100.frc2025.Swerve.Auto;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.FieldConstants.CoralStation;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Coral2Auto extends SequentialCommandGroup100 {
    public Coral2Auto(
            LoggerFactory logger,
            Wrist2 wrist, Elevator elevator,
            Funnel funnel,
            CoralTunnel tunnel,
            AlgaeGrip grip,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {
        super(logger);
        addCommands(
                // First Coral
                new ParallelCommandGroup100(
                        logger,
                        new Embark(
                                m_drive,
                                heedRadiusM,
                                controller,
                                profile,
                                FieldSector.IJ,
                                ReefDestination.RIGHT,
                                () -> ScoringPosition.L4),
                        new ParallelRaceGroup(
                                new WaitCommand(2),
                                new RunFunnelHandoff(logger, elevator, wrist, funnel, tunnel, grip))),
                new ScoreL4(logger, wrist, elevator),
                // Second Coral
                new ParallelRaceGroup( // NAVIGATOR DOSENT END? CHECK LOGS
                        new WaitCommand(3),
                        new GoToCoralStation(
                                logger,
                                m_drive,
                                controller,
                                viz,
                                kinodynamics,
                                CoralStation.Left,
                                0.5),
                        new RunFunnelHandoff(logger, elevator, wrist, funnel, tunnel, grip)),
                new ParallelRaceGroup(
                        new WaitCommand(0.1),
                        new RunFunnelHandoff(logger, elevator, wrist, funnel, tunnel, grip)),
                new Embark(m_drive, heedRadiusM,
                        controller, profile, FieldSector.KL, ReefDestination.LEFT,
                        () -> ScoringPosition.L4),
                new ScoreL4(logger, wrist, elevator));
    }
}
