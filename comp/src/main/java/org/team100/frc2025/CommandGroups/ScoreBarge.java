// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.IntakeAlgaeGrip;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreBarge extends SequentialCommandGroup {
  /** Creates a new ScoreBarge. */
  public ScoreBarge(Elevator elevator, Wrist2 wrist, AlgaeGrip algae) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(
            new SetElevator(elevator, 54, false), 
            new SetWrist(wrist, 3.4, true),
            new IntakeAlgaeGrip(algae, true)
            // new RunAlgaeGrip(algae)
        ),
        new ParallelCommandGroup(
            new SetElevatorPerpetually(elevator, 54) 
            // new OuttakeAlgaeGrip(wrist, algae)
        )




    );

    // new OuttakeAlgaeGrip(wrist, algae)

  }
}
