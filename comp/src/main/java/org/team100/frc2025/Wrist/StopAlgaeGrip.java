// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopAlgaeGrip extends Command {
  /** Creates a new StopAlgaeGrip. */
  AlgaeGrip m_grip;
  boolean m_perpetual = false;
  double position = 0;
  boolean hasAlgae = false;
  public StopAlgaeGrip(AlgaeGrip grip, boolean perpetual) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_grip = grip;
    m_perpetual = perpetual;
    addRequirements(m_grip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = 0;
    hasAlgae = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_grip.hasAlgae()){
        m_grip.setDutyCycle(-1);
        // position = m_grip.getPosition();
    } else {
        hasAlgae = true;
    }


    if(hasAlgae){
        // System.out.println("*************HAS ALGAE****************");
        // m_grip.setHoldPosition(position);
        // m_grip.setPosition(position);
        // m_grip.setDutyCycle(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_grip.setDutyCycle(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
