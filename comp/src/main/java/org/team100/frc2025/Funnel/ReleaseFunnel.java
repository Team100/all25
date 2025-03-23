// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Funnel;

import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Climber.ClimberRotate;
import org.team100.frc2025.Climber.SetClimber;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReleaseFunnel extends SequentialCommandGroup {
  /** Creates a new ReleaseFunnel. */
  public ReleaseFunnel(Funnel funnel, Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(
            new SetFunnelLatch(funnel, 180, 0),
            new WaitCommand(1)
        ),
        new ParallelRaceGroup(
            new SetClimber(climber, -1.42),
            new WaitCommand(1))
    );
  }
}
