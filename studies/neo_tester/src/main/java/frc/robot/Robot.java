
package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private final SparkMax motor;

  public Robot() {
    SparkMaxConfig conf = new SparkMaxConfig();
    conf.smartCurrentLimit(10, 10);
    motor = new SparkMax(5, MotorType.kBrushless);
    motor.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.startServer();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    motor.set(0.1);
    SmartDashboard.putNumber("amps", motor.getOutputCurrent());

  }

  @Override
  public void teleopExit() {
  }

}
