// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import java.util.function.BooleanSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrip extends SubsystemBase implements Glassy {
    private static final int currLim = 20;
    private static final double wheelDiameterM = 0.072;

    private final BareMotor m_motor;
    private final Neo550CANSparkMotor rawMotor;

    private final LinearMechanism m_linearMechanism;
    private final BooleanSupplier m_leftLimitSwitch;
    private final BooleanSupplier m_rightLimitSwitch;
    private final BooleanLogger m_logLeftLimitSwitch;
    private final BooleanLogger m_logRightLimitSwitch;
    public double position;

    private static final double m_5to1 = 5.2307692308;

    public AlgaeGrip(LoggerFactory parent) {
        LoggerFactory child = parent.child(this);

        m_logRightLimitSwitch = child.booleanLogger(Level.TRACE, "right");
        m_logLeftLimitSwitch = child.booleanLogger(Level.TRACE, "left");
        switch (Identity.instance) {
            case COMP_BOT -> {
                Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                        child,
                        25,
                        MotorPhase.FORWARD,
                        currLim,
                        Feedforward100.makeNeo550(),
                        PIDConstants.makePositionPID(0.1));
                m_leftLimitSwitch = motor::getForwardLimitSwitch;
                m_rightLimitSwitch = motor::getReverseLimitSwitch;
                m_linearMechanism = new SimpleLinearMechanism(
                        motor,
                        new CANSparkEncoder(child, motor),
                        m_5to1 * m_5to1,
                        wheelDiameterM);
                m_motor = motor;
                rawMotor = motor;
                reset();
            }
            default -> {
                SimulatedBareMotor motor = new SimulatedBareMotor(child, 100);
                // TODO: expose these switches so the command can work in simulation
                m_leftLimitSwitch = () -> false;
                m_rightLimitSwitch = () -> false;
                m_linearMechanism = new SimpleLinearMechanism(
                        motor,
                        new SimulatedBareEncoder(child, motor),
                        m_5to1 * m_5to1,
                        wheelDiameterM);
                m_motor = motor;
                rawMotor = null;
            }
        }

    }

    public void reset() {
        m_linearMechanism.resetEncoderPosition();
        position = 0;
    }

    @Override
    public void periodic() {
        m_logRightLimitSwitch.log(() -> rawMotor.getForwardLimitSwitch());
        m_logLeftLimitSwitch.log(() -> rawMotor.getReverseLimitSwitch());
        // This method will be called once per scheduler run
    }

    public boolean hasAlgae() {
        return m_rightLimitSwitch.getAsBoolean() || m_leftLimitSwitch.getAsBoolean();
    }

    public void intake() {
        if (!hasAlgae()) {
            position = m_linearMechanism.getPositionM().getAsDouble();
            m_linearMechanism.setDutyCycle(1);
            return;
        }
        // TODO get torque constant
        m_motor.setPosition(position, 0, 0, 0.1);
    }


    public void setDutyCycle(){
        m_motor.setDutyCycle(1);
    }
    public void stop() {
        m_motor.setDutyCycle(0);
    }

    public void outtake() {
        m_linearMechanism.setDutyCycle(-1);
    }
}
