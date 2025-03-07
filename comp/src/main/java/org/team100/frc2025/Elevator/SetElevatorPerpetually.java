// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPerpetually extends Command {
  /** Creates a new SetElevator. */
  Elevator m_elevator;
  double m_value;
  boolean finished = false;
  public SetElevatorPerpetually(Elevator elevator, double value) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_value = value;
    finished = false;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.resetElevatorProfile();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setPosition(m_value); //24.5 for l3

    double error = Math.abs(m_elevator.getPosition() - m_value);
    if(error < 0.5){
        finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
    finished = false;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
