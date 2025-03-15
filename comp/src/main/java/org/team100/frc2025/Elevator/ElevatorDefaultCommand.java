// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Elevator;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDefaultCommand extends Command {
  /** Creates a new ElevatorDefaultCommand. */
  Elevator m_elevator;
  Wrist2 m_wrist;
  AlgaeGrip m_grip;
  SwerveDriveSubsystem m_drive;
  public ElevatorDefaultCommand(Elevator elevator, Wrist2 wrist, AlgaeGrip grip, SwerveDriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_wrist = wrist;
    m_grip = grip;
    m_drive = drive;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.resetElevatorProfile();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double distanceToReef = FieldConstants.getDistanceToReefCenter(m_drive.getPose().getTranslation());

    if(distanceToReef > 1.6){

        if(!m_grip.hasAlgae()){
            double goal = 0.05;

            if(m_wrist.getSafeCondition()){
                m_elevator.setPositionNoGravity(goal);
            } else {
                m_elevator.setStatic();
            }

            double error = Math.abs(m_elevator.getPosition() - goal);

            if(error <= 0.3){
                m_elevator.setSafeCondition(true);
                // m_elevator.setPosition(goal);


            } else{
                m_elevator.setSafeCondition(false);
                // m_elevator.setPosition(goal);

            }
        } else{
            double goal = 12;

            if(m_wrist.getSafeCondition()){
                m_elevator.setPosition(goal);
            } else {
                m_elevator.setStatic();
            }

            // double error = Math.abs(m_elevator.getPosition() - goal);

            // if(error <= 0.2){
            //     m_elevator.setSafeCondition(true);
            // } else{
            //     m_elevator.setSafeCondition(false);
            // }
            
        }
    } else {
        m_elevator.setStatic();
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
