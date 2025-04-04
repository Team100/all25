package org.team100.frc2025.Swerve.Auto;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.FieldConstants.CoralStation;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.CommandGroups.ApproachL4;
import org.team100.frc2025.CommandGroups.PostDropCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevatorNotTimed;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

public class FancyAuto extends SequentialCommandGroup100 {

    public FancyAuto(
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
        super(logger, "FancyAuto");
        addCommands(
                new ParallelCommandGroup100(m_logger, "embark1",
                        new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.IJ,
                                ReefDestination.RIGHT,
                                () -> ScoringPosition.L4),
                        new SequentialCommandGroup100(m_logger, "getCoralOut",
                                new ParallelCommandGroup100(m_logger, "coralDown",
                                        new SetWrist(wrist, 0.4, false),
                                        new SetElevatorNotTimed(m_logger, elevator, 0, false)),
                                new ApproachL4(m_logger, wrist, elevator))),
                new PostDropCoralL4(wrist, elevator, 10),
                new ParallelDeadlineGroup100(m_logger, "pick1",
                        new GoToCoralStation(m_logger, m_drive, controller, viz, kinodynamics, CoralStation.Left, 0.5),
                        new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)),
                new ParallelCommandGroup100(m_logger, "embark2",
                        new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.KL,
                                ReefDestination.LEFT,
                                () -> ScoringPosition.L4),
                        new ApproachL4(m_logger, wrist, elevator)),
                new PostDropCoralL4(wrist, elevator, 10),
                new ParallelDeadlineGroup100(m_logger, "pick2",
                        new GoToCoralStation(m_logger, m_drive, controller, viz, kinodynamics, CoralStation.Left, 0.5),
                        new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)),
                new ParallelCommandGroup100(m_logger, "embark3",
                        new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.KL,
                                ReefDestination.RIGHT   ,
                                () -> ScoringPosition.L4),
                        new ApproachL4(m_logger, wrist, elevator)),
                new PostDropCoralL4(wrist, elevator, 10)
        );
    }
}
