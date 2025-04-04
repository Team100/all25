package org.team100.frc2025.Swerve.AutoOld;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.CancelCommand;
import org.team100.frc2025.CommandGroups.DeadlineForEmbarkAndPrePlace;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreL4;
import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.HoldWristAndElevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.Auto.GoToCoralStation;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.CoralStation;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.ParallelRaceGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Coral2AutoLeft extends SequentialCommandGroup100 {
    public Coral2AutoLeft(
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
        super(logger, "Coral2Auto");


        Embark embarkToI = new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.IJ, ReefDestination.LEFT,
        () -> ScoringPosition.L4, ReefPoint.I, true);

        Embark embarkToK = new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.KL, ReefDestination.LEFT,
        () -> ScoringPosition.L4, ReefPoint.K, true);

        Embark embarkToL = new Embark(m_logger, m_drive, heedRadiusM, controller, profile, FieldSector.KL, ReefDestination.RIGHT,
        () -> ScoringPosition.L4, ReefPoint.L, true);
        
        PrePlaceCoralL4 prePlaceCoralL4I = new PrePlaceCoralL4(wrist, elevator, 47, true);

        PrePlaceCoralL4 prePlaceCoralL4K = new PrePlaceCoralL4(wrist, elevator, 47, true);

        PrePlaceCoralL4 prePlaceCoralL4L = new PrePlaceCoralL4(wrist, elevator, 47, true);


        addCommands(
                new ParallelDeadlineGroup100(
                    m_logger, 
                    "embark1",
                    new DeadlineForEmbarkAndPrePlace( embarkToI::isDone, prePlaceCoralL4I::isDone),
                    embarkToI,
                    new SequentialCommandGroup100(logger, "handoff then place",      
                        new ParallelRaceGroup100(m_logger, "handoff",
                            new WaitCommand(0.5)
                            // new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)
                        ),
                        new SetWrist(wrist, 0.4, false),
                        prePlaceCoralL4I
                    )
                ),
                new PostDropCoralL4(wrist, elevator, 10),

                new ParallelDeadlineGroup100(
                    logger, 
                    "Pick up", 
                    new GoToCoralStation(logger, m_drive, controller, viz, kinodynamics, CoralStation.Left, 0.5), 
                    new RunFunnelHandoff(logger, elevator, wrist, funnel, tunnel, grip)),

                new ParallelDeadlineGroup100(
                    m_logger, 
                    "embark1",
                    new DeadlineForEmbarkAndPrePlace( embarkToK::isDone, prePlaceCoralL4K::isDone),
                    embarkToK,
                    new SequentialCommandGroup100(logger, "handoff then place",      
                        new ParallelRaceGroup100(m_logger, "handoff",
                            new WaitCommand(0.5)
                            // new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)
                        ),
                        new SetWrist(wrist, 0.4, false),
                        prePlaceCoralL4K
                    )
                ),
                new PostDropCoralL4(wrist, elevator, 10),


                new ParallelDeadlineGroup100(
                    logger, 
                    "Pick up", 
                    new GoToCoralStation(logger, m_drive, controller, viz, kinodynamics, CoralStation.Left, 0.5), 
                    new RunFunnelHandoff(logger, elevator, wrist, funnel, tunnel, grip)),

                new ParallelDeadlineGroup100(
                    m_logger, 
                    "embark1",
                    new DeadlineForEmbarkAndPrePlace( embarkToL::isDone, prePlaceCoralL4L::isDone ),
                    embarkToL,
                    new SequentialCommandGroup100(logger, "handoff then place",      
                        new ParallelRaceGroup100(m_logger, "handoff",
                            new WaitCommand(0.5)
                            // new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)
                        ),
                        new SetWrist(wrist, 0.4, false),
                        prePlaceCoralL4L
                    )
                ),
                new PostDropCoralL4(wrist, elevator, 10)

                // embarkToI,
                // new GoToCoralStation(logger, m_drive, controller, viz, kinodynamics, CoralStation.Left, 0.5), 
                // embarkToK,
                // new GoToCoralStation(logger, m_drive, controller, viz, kinodynamics, CoralStation.Left, 0.5), 
                // embarkToL
                

        
        

        
        );
                
    }
}
