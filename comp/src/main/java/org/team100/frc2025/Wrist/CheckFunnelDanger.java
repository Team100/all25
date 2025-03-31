// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import java.io.FileInputStream;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CheckFunnelDanger extends Command {
  /** Creates a new CheckFunnelDanger. */
  Wrist2 m_wrist;
  Elevator m_elevator;
  double m_initialElevatorValue;
  boolean finished = false;
  public CheckFunnelDanger(Wrist2 wrist, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_elevator = elevator;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialElevatorValue = m_elevator.getPosition();
    finished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_wrist.getAngle() > 1.6){
        m_wrist.setAngleValue(1.5);
        m_elevator.setPosition(m_initialElevatorValue);
    } else {
        finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
