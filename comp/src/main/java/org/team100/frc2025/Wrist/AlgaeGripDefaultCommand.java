// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeGripDefaultCommand extends Command {
  /** Creates a new AlgaeGripDefaultCommand. */
  AlgaeGrip m_grip;
  public AlgaeGripDefaultCommand(AlgaeGrip grip) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_grip = grip;
    addRequirements(m_grip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_grip.hasAlgae()){
        m_grip.setDutyCycle(0);
    }else{
        m_grip.setDutyCycle(1);
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
