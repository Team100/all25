
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  GenericHID hid = new GenericHID(0);

  public Robot() {

  }

  @Override
  public void robotPeriodic() {
    for (int i = 0; i < 32; ++i) {
      System.out.printf("%d ", hid.getRawButton(i)?1:0);
    }
    System.out.println();
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

}
