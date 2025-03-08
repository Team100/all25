// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550Factory;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrip extends SubsystemBase implements Glassy {
  /** Creates a new CoralTunnel. */
  // private final LinearMechanism m_coralMech;
  private LaserCan laserCAN;

  // private final SparkMax m_motor;
  private final LinearMechanism m_algaeMech;

  public AlgaeGrip(LoggerFactory parent, int algaeID) {

    LoggerFactory child = parent.child(this);
    int currLim = 20;

    m_algaeMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, currLim, 3, 1,
        MotorPhase.FORWARD, 1);

    // m_motor = new SparkMax(25, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setAlgaeMotor(double value) {
    // m_coralMech.setDutyCycle(value);

    m_algaeMech.setDutyCycle(value);
  }
}
