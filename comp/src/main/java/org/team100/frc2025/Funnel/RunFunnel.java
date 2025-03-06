// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Funnel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunFunnel extends Command {
  /** Creates a new RunFunnel. */
  Funnel m_funnel;
  Timer timer = new Timer();

  public RunFunnel(Funnel funnel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_funnel = funnel;
    addRequirements(m_funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < 0.75){
        m_funnel.setFunnel(0.70);
      } else if(timer.get() < 0.85){
        m_funnel.setFunnel(-0.2);
      } else{
        timer.restart();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_funnel.setFunnel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
