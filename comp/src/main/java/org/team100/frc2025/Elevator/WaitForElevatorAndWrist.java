// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Elevator;

import java.io.FileInputStream;

import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForElevatorAndWrist extends Command {
  /** Creates a new WaitForElevatorAndWrist. */
  Elevator m_elevator;
  Wrist2 m_wrist;
  boolean finished = false;
  public WaitForElevatorAndWrist(Elevator elevator, Wrist2 wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_wrist = wrist;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_elevator.getPosition() <= 20 && m_wrist.getAngle() <= 0.9){
        finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
