// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Wrist.ElevatorDutyCycle;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.SetWristDutyCycle;
import org.team100.frc2025.Wrist.SetWristHandoff;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareFunnelHandoff extends SequentialCommandGroup {
  /** Creates a new SetFunnelHandoff. */
  Wrist2 m_wrist;
  Elevator m_elevator;

  public PrepareFunnelHandoff(Wrist2 wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_wrist = wrist;
    addCommands(
        new SetWrist(wrist, 0.1, false),
        new SetElevatorFunnelHandoff(elevator, 0.1),
        new ParallelRaceGroup(new WaitCommand(0.5), new ElevatorDutyCycle(elevator)),
        new SetWristHandoff(wrist, 0.1),
        new SetWristDutyCycle(wrist, -0.15, false)

    );
  }
}
