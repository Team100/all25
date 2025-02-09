
package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private final TalonFX extend;
  private final PositionVoltage positionCommand;
  private final SparkMax roller;
  private final XboxController controller;
  private final TrapezoidProfile profile;

  private State state;

  public Robot() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startServer();
    Constraints constraints = new Constraints(14,55);
    profile = new TrapezoidProfile(constraints);

    extend = new TalonFX(2);
    TalonFXConfigurator talonFXConfigurator = extend.getConfigurator();
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimit = 80;
    currentConfigs.SupplyCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = 110;
    currentConfigs.StatorCurrentLimitEnable = false;
    talonFXConfigurator.apply(currentConfigs, 0.1);
    //save

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.0;
    slot0Configs.kP = 10;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0 ;
    talonFXConfigurator.apply(slot0Configs, 0.1);



    positionCommand = new PositionVoltage(0).withEnableFOC(false);

    SparkMaxConfig rollerConf = new SparkMaxConfig();
    rollerConf.smartCurrentLimit(20, 0);
    roller = new SparkMax(5, MotorType.kBrushless);
    roller.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = new XboxController(0);
    state = new State(0, 0);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("supply", extend.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("stator", extend.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("position", extend.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("duty cycle", extend.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("roller amps", roller.getOutputCurrent());
    SmartDashboard.putNumber("roller speeed", roller.getEncoder().getVelocity());
    SmartDashboard.putNumber("stateP", state.position);
    SmartDashboard.putNumber("stateV", state.velocity); 
  }

  @Override
  public void teleopInit() {
    extend.setPosition(0);
    state = new State(0, 0);
  }

  @Override
  public void teleopPeriodic() {
    if (controller.getXButton()) {
      state = profile.calculate(kDefaultPeriod, state, new State(5, 0));
      extend.setControl(positionCommand.withPosition(state.position));
      SmartDashboard.putBoolean("out", true);     
    } else {
      state = profile.calculate(kDefaultPeriod, state, new State(1.5, 0));
      extend.setControl(positionCommand.withPosition(state.position));
      SmartDashboard.putBoolean("out", false);
    }
    if (controller.getYButton()) {
      roller.set(1);
      SmartDashboard.putBoolean("roll", true);
    } else {
      roller.set(0.0);
      SmartDashboard.putBoolean("roll", true);
    }
  }

  @Override
  public void teleopExit() {
  }

}
