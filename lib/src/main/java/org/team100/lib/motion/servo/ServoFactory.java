package org.team100.lib.motion.servo;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.NeoVortexCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;

public class ServoFactory {

    public static LimitedLinearVelocityServo limitedNeoVelocityServo(
            LoggerFactory parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            double wheelDiameter,
            double maxVelocity,
            double maxAccel,
            double maxDecel,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoCANSparkMotor motor = new NeoCANSparkMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                ff,
                lowLevelVelocityConstants);
        CANSparkEncoder encoder = new CANSparkEncoder(
                parent,
                motor);
        LinearMechanism mech = new SimpleLinearMechanism(
                motor,
                encoder,
                gearRatio,
                wheelDiameter);
        LinearVelocityServo v = new OutboardLinearVelocityServo(
                parent,
                mech);
        return new LimitedLinearVelocityServo(v,
                maxVelocity,
                maxAccel,
                maxDecel);
    }

    public static LimitedLinearVelocityServo limitedSimulatedVelocityServo(
            LoggerFactory parent,
            double gearRatio,
            double wheelDiameterM,
            double maxVelocity,
            double maxAccel,
            double maxDecel) {
        // motor speed is rad/s
        BareMotor motor = new SimulatedBareMotor(parent, 600);
        LinearMechanism mech = new SimpleLinearMechanism(
                motor,
                new SimulatedBareEncoder(parent, motor),
                gearRatio,
                wheelDiameterM);
        LinearVelocityServo v = new OutboardLinearVelocityServo(
                parent,
                mech);
        return new LimitedLinearVelocityServo(v,
                maxVelocity,
                maxAccel,
                maxDecel);
    }

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     */
    public static AngularPositionServo neoVortexAngleServo(
            LoggerFactory parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            double maxVelocity,
            double maxAccel,
            Feedback100 controller,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        CANSparkMotor motor = new NeoVortexCANSparkMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                ff,
                lowLevelVelocityConstants);
        RotaryMechanism mech = new SimpleRotaryMechanism(
                parent,
                motor,
                new CANSparkEncoder(parent, motor),
                gearRatio);
        RotaryPositionSensor sensor = new ProxyRotaryPositionSensor(mech);
        return new OnboardAngularPositionServo(
                parent,
                mech,
                sensor,
                new TrapezoidProfile100(maxVelocity, maxAccel, 0.05),
                controller);
    }

    public static AngularPositionServo simulatedAngleServo(
            LoggerFactory parent,
            double maxVelocity,
            double maxAccel,
            Feedback100 controller) {
        // motor speed is rad/s
        SimulatedBareMotor motor = new SimulatedBareMotor(parent, 600);
        RotaryMechanism mech = new SimpleRotaryMechanism(
                parent,
                motor,
                new SimulatedBareEncoder(parent, motor),
                1);
        RotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                parent,
                mech);
        // the new sim doesn't have hard stops; should it?
        // 0, // minimum hard stop
        // 2); // maximum hard stop
        return new OnboardAngularPositionServo(
                parent,
                mech,
                sensor,
                new TrapezoidProfile100(maxVelocity, maxAccel, 0.05),
                controller);
    }

    private ServoFactory() {
        //
    }
}
