package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {

  XboxController controller = new XboxController(0);

  TalonFX motor = new TalonFX(15);

  public Robot() {
    // intake
    new Trigger(controller::getXButton).whileTrue(new FunctionalCommand(
        this::lowCurrent,
        () -> motor.set(1),
        interrupted -> motor.stopMotor(),
        () -> false));
    // out
    new Trigger(controller::getYButton).whileTrue(new FunctionalCommand(
        this::highCurrent,
        () -> motor.set(-1),
        interrupted -> motor.stopMotor(),
        () -> false));

  }

  void lowCurrent() {
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimit = 40;
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = 40;
    currentConfigs.StatorCurrentLimitEnable = true;
    TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();
    talonFXConfigurator.apply(currentConfigs, 1);
  }

  void highCurrent() {
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimit = 60;
    currentConfigs.SupplyCurrentLimitEnable = false;
    currentConfigs.StatorCurrentLimit = 90;
    currentConfigs.StatorCurrentLimitEnable = false;
    TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();
    talonFXConfigurator.apply(currentConfigs, 1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    motor.stopMotor();
  }

}
