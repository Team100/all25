// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Funnel;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetFunnelLatch extends Command {
  /** Creates a new SetFunnelLatch. */
  Funnel m_funnel;
  double m_value1;
  double m_value2;

  public SetFunnelLatch(Funnel funnel, double value1, double value2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_funnel = funnel;
    m_value1 = value1;
    m_value2 = value2;
    addRequirements(m_funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_funnel.setLatch1(m_value1);
    m_funnel.setLatch2(m_value2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // double error1 = Math.abs(m_funnel.getLatch1() - m_value1);
    // double error2 = Math.abs(m_funnel.getLatch2() - m_value2);

    // if(error1 < 3 && error2 < 3){
    //     return true;
    // }

    // if(error1 < 3){
    //     return true;
    // }

    return false;
  }
}
