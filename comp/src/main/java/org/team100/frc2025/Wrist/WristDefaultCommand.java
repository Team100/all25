// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristDefaultCommand extends Command {
  /** Creates a new WristDefaultCommand. */
  Elevator m_elevator;
  Wrist2 m_wrist;
  double deadband = 0.03;
  double count = 0;
  boolean docked = false;
  public WristDefaultCommand(Wrist2 wrist, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_wrist = wrist;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.resetWristProfile();
    count = 0;
    docked = false;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_elevator.getPosition() > 17.5){
        m_wrist.setAngleValue(0.5);
      if(m_wrist.getAngle() < 0.5 - deadband){
        m_wrist.setSafeCondition(false);
        // m_wrist.setAngleValue(0.5);
      } else if(m_wrist.getAngle() > 0.5 - deadband && m_wrist.getAngle() < 1.78 + deadband){
        m_wrist.setSafeCondition(true);
        // m_wrist.setAngleValue(0.5);
      } else if(m_wrist.getAngle() > 1.78){
        m_wrist.setSafeCondition(false);
        // m_wrist.setAngleValue(0.5);
      }
    } else if(m_elevator.getPosition() > 2 && m_elevator.getPosition() < 17.5){
        m_wrist.setAngleValue(0.5);
      if(m_wrist.getAngle() < 0.5 - deadband){
        m_wrist.setSafeCondition(false);
        // m_wrist.setAngleValue(0.5);
      } else if(m_wrist.getAngle() > 0.5 - deadband && m_wrist.getAngle() < 1.78 + deadband ){
        m_wrist.setSafeCondition(true);
        // m_wrist.setAngleValue(0.5);
      } else if(m_wrist.getAngle() > 1.78){
        m_wrist.setSafeCondition(false);
        // m_wrist.setAngleValue(0.5);
      }
    } else {
        if(m_elevator.getSafeCondition()){
            m_wrist.setSafeCondition(true);
            m_wrist.setAngleValue(0.1);
        }
    }


    // if(m_elevator.getSafeCondition()){
    //     m_wrist.setAngleValue(0.1);
    // }





    

    

    


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
