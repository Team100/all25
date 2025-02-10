// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  TalonFX starboard = new TalonFX(0);
  TalonFX port = new TalonFX(1);

  public ExampleSubsystem() {

    double supply = 10;
    double stator = 10;

    TalonFXConfigurator starboardConfig = starboard.getConfigurator();
    TalonFXConfigurator portConfig = port.getConfigurator();

    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimit = supply;
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = stator;
    currentConfigs.StatorCurrentLimitEnable = true;

    starboardConfig.apply(currentConfigs);
    portConfig.apply(currentConfigs);
  }

  public void setDuty(double value){
    starboard.set(value);
    port.set(value);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public void run() {
    // Query some boolean state, such as a digital sensor.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // neo550.set(0.3);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
