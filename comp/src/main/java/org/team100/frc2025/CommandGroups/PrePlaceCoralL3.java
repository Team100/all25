// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrePlaceCoralL3 extends Command {
  /** Creates a new PrePlaceCoralL3. */
    Wrist2 m_wrist;
  Elevator m_elevator;
  double m_elevatorGoal;
  double count = 0;
  boolean finished = false;

  public PrePlaceCoralL3(Wrist2 wrist, Elevator elevator, double elevatorValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_elevator = elevator;
    m_elevatorGoal = elevatorValue;
    addRequirements(m_wrist, m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    finished = false;
    m_wrist.resetWristProfile();
    m_elevator.resetElevatorProfile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setPosition(m_elevatorGoal);
    if(m_elevatorGoal - 10 > m_elevator.getPosition()){
      m_wrist.setAngleValue(0.4);
    } else {
      m_wrist.setAngleValue(0.9);
    }

    double error = Math.abs(m_elevator.getPosition() - m_elevatorGoal);

    if(error < 0.5){
      count++;
    } else {
      count = 0;
    }

    if(count >= 20){
      finished = true;
    }
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
