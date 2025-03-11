// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralTunnel extends SubsystemBase implements Glassy {
    /** Creates a new CoralTunnel. */
    // private final LinearMechanism m_coralMech;
    private LaserCan laserCAN;

    private final BareMotor m_motor;

    public CoralTunnel(LoggerFactory parent, int algaeID, int coralID) {

        LoggerFactory child = parent.child(this);
        int coralCurrentLimit = 20;

        switch (Identity.instance) {
            case COMP_BOT -> {
                // m_coralMech = Neo550Factory.getNEO550LinearMechanism(getName(), child,
                // coralCurrentLimit, coralID, 1,
                // MotorPhase.FORWARD, 1);
                // m_motor = new SparkMax(25, MotorType.kBrushless);
                m_motor = new Neo550CANSparkMotor(
                        child,
                        25,
                        MotorPhase.FORWARD,
                        coralCurrentLimit,
                        Feedforward100.makeNeo550(),
                        new PIDConstants());
            }
            default -> {
                m_motor = new SimulatedBareMotor(child, 100);
            }
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setAlgaeMotor(double value) {
        // m_algaeMech.setDutyCycle(value);
    }

    public void setCoralMotor(double value) {
        // m_coralMech.setDutyCycle(value);

        m_motor.setDutyCycle(value);
    }
}
