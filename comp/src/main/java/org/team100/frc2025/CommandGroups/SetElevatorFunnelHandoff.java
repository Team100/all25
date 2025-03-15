// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorFunnelHandoff extends Command {
  /** Creates a new SetElevator. */
  Elevator m_elevator;
  double m_value;
  boolean finished = false;
  double count = 0;
  public SetElevatorFunnelHandoff(Elevator elevator, double value) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_value = value;
    finished = false;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    finished = false;
    m_elevator.resetElevatorProfile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setPositionNoGravity(m_value); //24.5 for l3

    double error = Math.abs(m_elevator.getPosition() - m_value);
    if(error < 1){
        count++;
    } else{
        count = 0;
    }

    if(count >= 20){
        finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
    finished = false;
    count = 0;
    System.out.println("I FINISHED NUMBER 1");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        return finished;
  }
}
