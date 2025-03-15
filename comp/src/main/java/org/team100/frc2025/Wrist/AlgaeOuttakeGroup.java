// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import java.lang.annotation.ElementType;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeOuttakeGroup extends ParallelCommandGroup {
  /** Creates a new AlgaeOuttakeGroup. */
  public AlgaeOuttakeGroup(AlgaeGrip grip, Wrist2 wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new OuttakeAlgaeGrip(grip),
        new SetWrist(wrist, 3.7, true),
        new SetElevatorPerpetually(elevator, 12)
    );
  }
}
