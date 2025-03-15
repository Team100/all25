// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Swerve.Auto;

import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.team100.frc2025.FieldConstants.CoralStation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Coral2Auto extends SequentialCommandGroup {
  /** Creates a new Coral2Auto. */
  public Coral2Auto(LoggerFactory logger, Wrist2 wrist, Elevator elevator, Funnel funnel, CoralTunnel tunnel, AlgaeGrip grip, SwerveController controller, HolonomicProfile profile, SwerveDriveSubsystem m_drive, SwerveKinodynamics kinodynamics, TrajectoryVisualization viz) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        //First Coral
        new ParallelCommandGroup(
            new Embark(m_drive, controller, profile, FieldSector.IJ, ReefDestination.RIGHT, () -> ScoringPosition.L4),
            new ParallelRaceGroup(
                new WaitCommand(2),
                new RunFunnelHandoff(elevator, wrist, funnel, tunnel, grip)
            )
        ),
        new ScoreL4(wrist, elevator),
        //Second Coral
        new ParallelRaceGroup( //NAVIGATOR DOSENT END? CHECK LOGS
            new WaitCommand(3),
            new GoToCoralStation(
                logger,
                m_drive,
                controller,
                viz,
                kinodynamics,
                CoralStation.Left,
                0.5
            ),
            new RunFunnelHandoff(elevator, wrist, funnel, tunnel, grip)
        ),
        new ParallelRaceGroup(
            new WaitCommand(1.5),
            new RunFunnelHandoff(elevator, wrist, funnel, tunnel, grip)
        ),
        new Embark(m_drive, controller, profile, FieldSector.KL, ReefDestination.LEFT, () -> ScoringPosition.L4),
        
        new ScoreL4(wrist, elevator)

        
    );
  }
}
