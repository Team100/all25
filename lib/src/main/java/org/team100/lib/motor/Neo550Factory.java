package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;

public class Neo550Factory {

    public static LinearMechanism getNEO550LinearMechanism(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        LoggerFactory moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                moduleLogger,
                canID,
                motorPhase,
                currentLimit,
                Feedforward100.makeNeo550(),
                new PIDConstants());
        CANSparkEncoder encoder = new CANSparkEncoder(moduleLogger, motor);
        return new LinearMechanism(
                motor, encoder, gearRatio, wheelDiameterM, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public static RotaryMechanism getNEO550RotaryMechanism(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase) {
        LoggerFactory moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                moduleLogger,
                canID,
                motorPhase,
                currentLimit,
                Feedforward100.makeNeo550(),
                PIDConstants.makePositionPID(1));
        CANSparkEncoder encoder = new CANSparkEncoder(moduleLogger, motor);
        ProxyRotaryPositionSensor sensor = new ProxyRotaryPositionSensor(encoder, gearRatio);
        return new RotaryMechanism(
                moduleLogger,
                motor,
                sensor,
                gearRatio,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public static LinearVelocityServo getNEO550VelocityServo(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        LoggerFactory moduleLogger = parent.child(name);
        return new OutboardLinearVelocityServo(moduleLogger, getNEO550LinearMechanism(name,
                moduleLogger,
                currentLimit,
                canID,
                gearRatio,
                motorPhase,
                wheelDiameterM));
    }

    public static OutboardLinearVelocityServo simulatedDriveServo(LoggerFactory parent) {
        return new OutboardLinearVelocityServo(
                parent,
                simulatedLinearMechanism(parent));
    }

    public static RotaryMechanism simulatedRotaryMechanism(LoggerFactory parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, driveMotor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(parent, encoder, 1, () -> 0);
        return new RotaryMechanism(
                parent, driveMotor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public static LinearMechanism simulatedLinearMechanism(LoggerFactory parent) {
        // simulated drive motor free speed is 5 m/s
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        // simulated gearing is 2 meter wheel, 1:1, so rad/s and m/s are the same.
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, driveMotor);
        return new LinearMechanism(
                driveMotor, encoder, 1, 2, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }
}
