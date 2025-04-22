// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Swerve.Auto;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.DeadlineForEmbarkAndPrePlace;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Coral1Mid extends SequentialCommandGroup {
  /** Creates a new Coral1Mid. */
  public Coral1Mid(
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
            TrajectoryVisualization viz
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Embark embarkToH = new Embark(logger, m_drive, heedRadiusM, controller, profile, FieldSector.GH,
                ReefDestination.LEFT,
                () -> ScoringPosition.L4, ReefPoint.H, true);
    
    PrePlaceCoralL4 prePlaceCoralL4H = new PrePlaceCoralL4(wrist, elevator, tunnel, 47, true);

 
    addCommands(
        new ParallelDeadlineGroup100(
                        logger,
                        "embark1",
                        new DeadlineForEmbarkAndPrePlace(embarkToH::isDone, prePlaceCoralL4H::isDone),
                        embarkToH,
                        new SequentialCommandGroup100(
                            logger, 
                            "handoff then place",
                            new ParallelRaceGroup100(logger, "handoff",
                                    new WaitCommand(0.5),
                                    new RunFunnelHandoff(logger, elevator, wrist, funnel, tunnel, grip)
                            ),
                            new ParallelCommandGroup100(logger, getName(), new SetWrist(wrist, 0.4, false), new SetElevator(logger, elevator, 1, false)),
                            prePlaceCoralL4H

                        )
        ),
        new PostDropCoralL4(wrist, elevator, 10)

    );
  }
}
