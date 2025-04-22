// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motor.SimulatedBareMotor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrip extends SubsystemBase implements Glassy {
    private static final int currLim = 10;
    private static final double wheelDiameterM = 0.072;

    private final BooleanLogger m_log_hasAlgae;

    CoralTunnel m_tunnel;
    // private final BareMotor m_motor;
    // private final Neo550CANSparkMotor rawMotor;

    // private final LinearMechanism m_linearMechanism;
    // private final BooleanSupplier m_leftLimitSwitch;
    // private final BooleanSupplier m_rightLimitSwitch;
    // private final BooleanLogger m_logLeftLimitSwitch;
    // private final BooleanLogger m_logRightLimitSwitch;
    public double m_holdPosition;

    TalonFX m_motor;
    // NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // NetworkTableEntry entry1;
    // NetworkTableEntry entry2;

    private static final double m_5to1 = 5.2307692308;


    public AlgaeGrip(LoggerFactory parent, CoralTunnel tunnel) {
        LoggerFactory child = parent.child(this);
        m_log_hasAlgae = child.booleanLogger(Level.TRACE, "has algae");
        m_tunnel = tunnel;

        // NetworkTable table = inst.getTable("My Table");

        // entry1 = table.getEntry("STATOR");
        // entry2 = table.getEntry("SUPPLY");

        // m_logRightLimitSwitch = child.booleanLogger(Level.TRACE, "right");
        // m_logLeftLimitSwitch = child.booleanLogger(Level.TRACE, "left");
        switch (Identity.instance) {
            case COMP_BOT -> {
                // Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                // child,
                // 25,
                // MotorPhase.FORWARD,
                // currLim,
                // Feedforward100.makeNeo550(),
                // PIDConstants.makePositionPID(0.1));
                // m_leftLimitSwitch = motor::getForwardLimitSwitch;
                // m_rightLimitSwitch = motor::getReverseLimitSwitch;
                // m_linearMechanism = new SimpleLinearMechanism(
                // motor,
                // new CANSparkEncoder(child, motor),
                // m_5to1 * m_5to1,
                // wheelDiameterM);
                // m_motor = motor;
                // rawMotor = motor;

                m_motor = new TalonFX(2);
                if(m_motor==null) return;
              
                TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
                CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();

                currentConfigs.SupplyCurrentLimit = 40;
                currentConfigs.SupplyCurrentLimitEnable = true;

                currentConfigs.StatorCurrentLimit = 40;
                currentConfigs.StatorCurrentLimitEnable = true;

                talonFXConfigurator.apply(currentConfigs);

                reset();
            }
            default -> {
                SimulatedBareMotor motor = new SimulatedBareMotor(child, 100);
                // TODO: expose these switches so the command can work in simulation
                // m_leftLimitSwitch = () -> false;
                // m_rightLimitSwitch = () -> false;
                // m_linearMechanism = new SimpleLinearMechanism(
                // motor,
                // new SimulatedBareEncoder(child, motor),
                // m_5to1 * m_5to1,
                // wheelDiameterM);
                // m_motor = motor;
                // rawMotor = null;
            }
        }

    }

    public void reset() {
        // m_linearMechanism.resetEncoderPosition();
        // m_holdPosition = 0;
    }

    @Override
    public void periodic() {
        // m_logRightLimitSwitch.log(() -> m_rightLimitSwitch.getAsBoolean());
        // m_logLeftLimitSwitch.log(() -> m_leftLimitSwitch.getAsBoolean());
        // m_linearMechanism.periodic();
        // This method will be called once per scheduler run
        if (m_motor == null)
            return;
        // entry1.setDouble(m_motor.getStatorCurrent().getValueAsDouble());
        // entry2.setDouble(m_motor.getSupplyCurrent().getValueAsDouble());

    }

    public boolean hasAlgae() {

        boolean hasAlgae = m_tunnel.hasAlgae();
        // m_log_hasAlgae.log(() -> hasAlgae);
        return hasAlgae;
    }

    // public void intake() {
    // if (!hasAlgae()) {
    // position = m_linearMechanism.getPositionM().getAsDouble();
    // m_linearMechanism.setDutyCycle(1);
    // return;
    // }
    // // TODO get torque constant
    // m_motor.setPosition(position, 0, 0, 0.1);
    // }

    public double getPosition() {
        // return m_linearMechanism.getPositionM().getAsDouble();
        return 0;
    }

    public void setPosition(double pos) {
        // m_motor.setPosition(pos, 0, 0, 0.1);

    }

    public void setDutyCycle(double dutyCycle) {
        if (m_motor == null)
            return;
        m_motor.set(-dutyCycle);
    }

    public void stop() {
        // m_motor.setDutyCycle(0);
    }

    public void outtake() {
        // m_linearMechanism.setDutyCycle(-1);
    }

    public void setHoldPosition(double position) {
        m_holdPosition = position;

    }

    public double getHoldPosition() {
        return m_holdPosition;

    }

    public void applyLowConfigs() {
        if (m_motor == null)
            return;
        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();

        currentConfigs.SupplyCurrentLimit = 35;
        currentConfigs.SupplyCurrentLimitEnable = true;

        currentConfigs.StatorCurrentLimit = 35;
        currentConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(currentConfigs);
    }

    public void applyHighConfigs() {
        if (m_motor == null)
            return;
        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();

        currentConfigs.SupplyCurrentLimit = 60;
        currentConfigs.SupplyCurrentLimitEnable = true;

        currentConfigs.StatorCurrentLimit = 90;
        currentConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(currentConfigs);
    }
}
