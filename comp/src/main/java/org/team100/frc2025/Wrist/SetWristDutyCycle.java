// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetWristDutyCycle extends Command {
  /** Creates a new SetWristDutyCycle. */
  Wrist2 m_wrist;
  double m_duty;
  boolean m_withCount;
  Timer m_timer;
  boolean isDone = false;

  public SetWristDutyCycle(Wrist2 wrist, double duty, boolean withCount) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_duty = duty;
    m_withCount = withCount;
    m_timer = new Timer();
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setWristDutyCycle(m_duty);

    if(m_timer.get() > 2){
        isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    isDone = false;

    // System.out.println("**************************************I FINISHED NUMBER 4*******************************************");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_withCount){
        return isDone;
    }
    return false;
  }
}
