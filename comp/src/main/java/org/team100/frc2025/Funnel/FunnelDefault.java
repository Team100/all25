// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Funnel;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FunnelDefault extends Command {
  /** Creates a new FunnelDefault. */
  Funnel m_funnel;
  public FunnelDefault(Funnel funnel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_funnel = funnel;
    addRequirements(m_funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_funnel.setFunnel(0);

    m_funnel.setLatch1(0);
    m_funnel.setLatch2(180);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
