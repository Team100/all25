// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025;

import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Funnel.RunFunnel;
import org.team100.frc2025.Wrist.RunAlgaeManipulator;
import org.team100.frc2025.Wrist.Wrist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Handoff extends ParallelCommandGroup {
  /** Creates a new Handoff. */
  Funnel m_funnel;
  Wrist m_wrist;

  public Handoff(Funnel funnel, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunAlgaeManipulator(wrist),
        new RunFunnel(funnel)
    );
  }
}
