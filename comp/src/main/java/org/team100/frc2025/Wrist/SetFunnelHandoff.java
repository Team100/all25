// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetFunnelHandoff extends SequentialCommandGroup {
  /** Creates a new SetFunnelHandoff. */
  Wrist2 m_wrist;
  public SetFunnelHandoff(Wrist2 wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_wrist = wrist;
    addCommands(
        new SetWrist(m_wrist),
        // new SetWristSafe(m_wrist)
        new SetWristDutyCycle(wrist)
    );
  }
}
