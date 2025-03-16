// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CheckWristDanger;
import org.team100.frc2025.Wrist.IntakeAlgaeGrip;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabAlgaeL2Dumb extends SequentialCommandGroup {
  /** Creates a new DescoreAlgaeDumb. */
  public GrabAlgaeL2Dumb(Wrist2 wrist, Elevator elevator, AlgaeGrip grip) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new SetAlgaeDescorePositionPrep(wrist, elevator),
        new CheckWristDanger(wrist),
        new ParallelDeadlineGroup(new SetElevator(elevator, 23, true), new SetWrist(wrist, 3.7, true), new IntakeAlgaeGrip(grip, true))

    );
  }
}
