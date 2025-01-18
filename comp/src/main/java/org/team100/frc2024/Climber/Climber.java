// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of

package org.team100.frc2024.Climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  TalonFX climberMotor = new TalonFX(2);


  CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
  TalonFXConfigurator talonFXConfigurator = climberMotor.getConfigurator();

  

  public Climber() {
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
    // currentConfigs.SupplyCurrentLimit = 10.0;
    // currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = 50.0;
    currentConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(currentConfigs);
    
  }

  public void setDutyCycle(double dutyCycle) {
    climberMotor.set(dutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
