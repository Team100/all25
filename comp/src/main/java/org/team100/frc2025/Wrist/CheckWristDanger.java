// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CheckWristDanger extends Command {
  /** Creates a new SetElevator. */
  Wrist2 m_wrist;
  double m_angle;
  boolean finished = false;
  double count = 0;
  public CheckWristDanger(Wrist2 wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    count = 0;
    m_wrist.resetWristProfile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_angle = 0.1;


    if(m_wrist.getAngle() < m_angle ){
        m_wrist.setAngleValue(m_angle);

        if(Math.abs(m_wrist.getAngle() - m_angle) < 0.05){
            count++;
        } else {
            count = 0;
        }

    } else {
        finished = true;
    }

    if(count >= 20){
        finished = true;
    }
    
   


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("**************************************I FINISHED*******************************************");
    m_wrist.setWristDutyCycle(0);
    finished = false;
    count = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_wrist.atSetpoint()
    return finished;
  }
}
