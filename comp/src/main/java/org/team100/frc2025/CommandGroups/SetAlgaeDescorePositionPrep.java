// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import java.util.Map;
import java.util.function.Supplier;


import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;

import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.frc2025.Wrist.SetWrist;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetAlgaeDescorePositionPrep extends SequentialCommandGroup {
  /** Creates a new ScoreLevel. */
  public SetAlgaeDescorePositionPrep(Wrist2 wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new RunAlgaeGrip(grip, 1)
        // new SetWrist(wrist, 0.5, false),
        // new SetElevator(elevator, 12, false),
        // new ParallelDeadlineGroup(new SetWrist(wrist, 3.7, false), new SetElevatorPerpetually(elevator, 12) )

    );

    // addCommands(new SetWrist(wrist, 1.25, true));
  }
}
