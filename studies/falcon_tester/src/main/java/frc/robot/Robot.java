
package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private final TalonFX motor;
  private final XboxController controller;
  private final PositionVoltage v;

  public Robot() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startServer();
    motor = new TalonFX(2);
    TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimit = 50;
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = 200;
    currentConfigs.StatorCurrentLimitEnable = false;
    StatusCode statusCode = talonFXConfigurator.apply(currentConfigs, 0.1);
    System.out.println(statusCode);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.0; // we use "arbitrary feedforward", not this.
    slot0Configs.kP = 10;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    statusCode = talonFXConfigurator.apply(slot0Configs, 0.1);
    System.out.println(statusCode);

    controller = new XboxController(0);

    v = new PositionVoltage(0).withEnableFOC(false);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void teleopInit() {
    motor.setPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    // motor.set(0.1);
    SmartDashboard.putNumber("supply", motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("stator", motor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("position", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("duty cycle", motor.getDutyCycle().getValueAsDouble());

    if (controller.getXButton()) {
      // motor.setControl(new VoltageOut(12));
      motor.setControl(v.withPosition(0.5));
      SmartDashboard.putBoolean("out", true);
    } else {
      // motor.setControl(new VoltageOut(0));
      motor.setControl(v.withPosition(0));
      SmartDashboard.putBoolean("out", false);
    }
  }

  @Override
  public void teleopExit() {
  }

}
