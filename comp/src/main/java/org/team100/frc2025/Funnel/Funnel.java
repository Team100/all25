// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Funnel;


import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardLinearPositionServo;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550Factory;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase implements Glassy {
  /** Creates a new Funnel. */

  LinearMechanism m_starboardMech;
  LinearMechanism m_portMech;

  public Funnel(
    LoggerFactory parent,
    int starboardID,
    int portID
  ) {
    LoggerFactory child = parent.child(this);


    int funnelSupplyLimit = 20;

    switch (Identity.instance) {
            case COMP_BOT -> {
                
                m_starboardMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, funnelSupplyLimit, starboardID, 1, MotorPhase.REVERSE, 1);
                m_portMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, funnelSupplyLimit, portID, 1, MotorPhase.FORWARD, 1);
                
                break;
            }
            default -> {
                

                m_starboardMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, funnelSupplyLimit, starboardID, 1, MotorPhase.FORWARD, 1);
                m_portMech = Neo550Factory.getNEO550LinearMechanism(getName(), child, funnelSupplyLimit, portID, 1, MotorPhase.FORWARD, 1);

            }
    }
  }

  public void setStarboard(double value){
    m_starboardMech.setDutyCycle(value);
  }

  public void setPort(double value){
    m_portMech.setDutyCycle(value);
  }

  public void setFunnel(double value){
    setStarboard(value);
    setPort(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
