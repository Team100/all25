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

  // TalonFX starboard = new TalonFX(1);
  // TalonFX port = new TalonFX(2);
  SparkMax number1Max ;
  SparkMax number2Max ;

  public ExampleSubsystem() {



    number1Max = new SparkMax(27, MotorType.kBrushless);
    SparkMaxConfig conf = new SparkMaxConfig();
    conf.smartCurrentLimit(20, 20);
    number1Max.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    number2Max = new SparkMax(5, MotorType.kBrushless);
    number2Max.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void set1(double value){
    // starboard.set(-value);
    // port.set(-value);

    number1Max.set(-value);
  }

  public void set2(double value){
    // starboard.set(-value);
    // port.set(-value);

    number2Max.set(-value);
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
