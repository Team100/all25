// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL3 extends SequentialCommandGroup {
  /** Creates a new ScoreL3. */
  public ScoreL3(Wrist2 wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetWrist(wrist, 0.4, false),
        new ParallelDeadlineGroup(new SetElevator(elevator, 23, false), new SetWrist(wrist, 0.4, true)),
        new ParallelDeadlineGroup(new SetWrist(wrist, 0.9, false), new SetElevatorPerpetually(elevator, 23)),
        new ParallelDeadlineGroup(new SetElevator(elevator, 16, false), new SetWrist(wrist, 0.9, true))

        // new ParallelDeadlineGroup(new SetElevator(elevator, 35, false), new SetWrist(wrist, 1.25, true))
    );
  }
}
