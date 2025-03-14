// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.SetWristDutyCycle;
import org.team100.frc2025.Wrist.SetWristHandoff;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PartRedSea extends SequentialCommandGroup {
  /** Creates a new PartRedSea. */
  public PartRedSea(Wrist2 wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetWristHandoff(wrist, 0.1),
        new SetWristDutyCycle(wrist, -0.11, true),
        new SetElevator(elevator, 10, false)
    );
  }
}
