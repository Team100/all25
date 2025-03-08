// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of

package org.team100.frc2025.Climber;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServoWithoutAbsolute;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  AngularPositionServo climberMotor;
  Falcon6Motor m_motor; 

  public Climber(LoggerFactory logger, int canID) {
    LoggerFactory child = logger.child("Climber");
    m_motor = new Falcon6Motor(child, canID, MotorPhase.FORWARD, 50, 50, PIDConstants.makePositionPID(1), Feedforward100.makeArmPivot());
    RotaryMechanism rotaryMechanism = new SimpleRotaryMechanism(child, m_motor, new Talon6Encoder(child, m_motor), 25*3*4);
    Profile100 profile100 = new TrapezoidProfile100(10, 10,.05);
    climberMotor = new OutboardAngularPositionServoWithoutAbsolute(child,rotaryMechanism ,profile100);
    // climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setDutyCycle(double dutyCycle) {
    m_motor.setDutyCycle(dutyCycle);
  }

  public void setAngle(){
    climberMotor.setPosition(18, 0);
  }

  public void reset(){
    climberMotor.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climberMotor.periodic();
  }
}
