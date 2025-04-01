// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreBargeSmart extends Command {
  /** Creates a new ScoreBargeSmart. */
  Elevator m_elevator;
  Wrist2 m_wrist;
  Supplier<Boolean> m_readyToShoot;
  AlgaeGrip m_grip;
  public ScoreBargeSmart(Elevator elevator, Wrist2 wrist, AlgaeGrip grip, Supplier<Boolean> readyToShoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_wrist = wrist;
    m_readyToShoot = readyToShoot;
    m_grip = grip;
    addRequirements(m_elevator, m_wrist, m_grip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_grip.applyHighConfigs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(!m_readyToShoot.get()) {
    //   m_wrist.setAngleValue(2.0);
    //   m_elevator.setPosition(30.0);

    //   m_grip.setDutyCycle(0.5);

    //     if(m_grip.hasAlgae()){
    //         m_grip.applyLowConfigs();
    //     }
    // } else {
      m_elevator.setPosition(57.0);
      m_wrist.setAngleValue(2.0);

    //   if(Math.abs(m_elevator.getPosition[]\() - 57) < 0.5){
        m_grip.applyHighConfigs();
        m_grip.setDutyCycle(-1);
    //   }
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
