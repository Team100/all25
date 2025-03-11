// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrip extends SubsystemBase implements Glassy {
    
    private final Neo550CANSparkMotor m_motor;
    private final LinearMechanism linearMechanism;
    public double position;

    private static final double m_5to1 = 5.2307692308;

    public AlgaeGrip(LoggerFactory parent) {

        LoggerFactory child = parent.child(this);
        int currLim = 1;

        switch (Identity.instance) {
            case COMP_BOT -> {
                m_motor = new Neo550CANSparkMotor(child, 3, MotorPhase.FORWARD, currLim, Feedforward100.makeNeo550(),
                        PIDConstants.makePositionPID(.1));
                linearMechanism = new SimpleLinearMechanism(m_motor,
                        new CANSparkEncoder(child, m_motor), m_5to1 * m_5to1, .072);
                reset();
            }
            default -> {
                linearMechanism = null;
                m_motor = null;
            }
        }

    }
    
    public void reset() {
        m_motor.resetEncoderPosition();
        position = 0; 
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public boolean hasAlgae() {
        return m_motor.getForwardLimitSwitch() || m_motor.getReverseLimitSwitch();
    }

    public void intake() {  
        if (!hasAlgae()) {
            position = linearMechanism.getPositionM().getAsDouble();
            linearMechanism.setDutyCycle(1);
            return;
        }
        // TODO get torque constant
        m_motor.setPosition(position,0,0, 0.1);
    }

    public void stop() {
        m_motor.setDutyCycle(0);
    }

    public void outtake() {
        linearMechanism.setDutyCycle(-1);
    }
}
