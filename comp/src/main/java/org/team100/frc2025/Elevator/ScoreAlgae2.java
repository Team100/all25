// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Elevator;

import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAlgae2 extends SequentialCommandGroup {
  /** Creates a new ScoreLevel. */
  public ScoreAlgae2(Wrist2 wrist, Elevator elevator, AlgaeGrip grip) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new RunAlgaeGrip(grip, 1)
        // new SetWrist(wrist, 0.5, false),
        new SetElevator(elevator, 12, false),
        new ParallelDeadlineGroup(new SetWrist(wrist, 3.7, true), new SetElevatorPerpetually(elevator, 12) )
        // new ParallelDeadlineGroup(new RunAlgaeGrip(grip, RunAlgaeGrip.GripState.INTAKE), new SetWrist(wrist, 3.7, true), new SetElevatorPerpetually(elevator, 10))

        // new SetElevator(elevator, 30, false),
        // new ParallelDeadlineGroup(new SetWrist(wrist, 1.2, false), new SetElevatorPerpetually(elevator, 57.2), new RunAlgaeGrip(grip, 1)),
        // new ParallelDeadlineGroup(new SetWrist(wrist, 2.3, false), new SetElevatorPerpetually(elevator, 57.2))
        // new ParallelDeadlineGroup(new SetElevator(elevator, 10, false), new SetWrist(wrist, 1.25, true)),
        // new RunAlgaeGrip(grip, -1)
        // new SetElevator(elevator, 2, false)

        // new ParallelDeadlineGroupnew SetElevator(elevator, 10, false), new SetWrist(wrist, 1.26, false))
        // new SetWrist(wrist, 0.5, false)

    );

    // addCommands(new SetWrist(wrist, 1.25, true));
  }
}
