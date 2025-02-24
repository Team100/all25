// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Intake;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase implements Glassy {
  /** Creates a new AlgaeIntake. */
  Neo550CANSparkMotor m_intake;
  public AlgaeIntake(LoggerFactory parent, int ID ) {
    LoggerFactory child = parent.child(this);

    m_intake = new Neo550CANSparkMotor(child, ID, MotorPhase.FORWARD, 30, Feedforward100.makeNeo550(), PIDConstants.makePositionPID(1));
  }

  public void setSpeed(double speed) {
    m_intake.setDutyCycle(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
