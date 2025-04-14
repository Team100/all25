package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.testing.Timeless;

public class UnprofiledOutboardAngularPositionServoTest implements Timeless {
    private static final double kDelta = 0.001;
    private final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 100);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1, () -> 0);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        UnprofiledOutboardAngularPositionServo servo = new UnprofiledOutboardAngularPositionServo(
                log, mech);

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), kDelta);
        assertEquals(0, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(0, sensor.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, mech.getVelocityRad_S().getAsDouble(), kDelta);

        servo.periodic();
        servo.setPositionSetpoint(new Setpoints1d(new Control100(0, 0), new Control100(1, 0)), 0);
        stepTime();

        // move 0 to 1 in 0.02 => v = 50
        assertEquals(50, motor.getVelocityRad_S(), kDelta);
        assertEquals(1, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(50, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(50, mech.getVelocityRad_S().getAsDouble(), kDelta);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(0.5, sensor.getPositionRad().getAsDouble(), kDelta);

        servo.periodic();
        servo.setPositionSetpoint(new Setpoints1d(new Control100(0, 0), new Control100(1, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionSetpoint(new Setpoints1d(new Control100(0, 0), new Control100(1, 0)), 0);
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), kDelta);
        assertEquals(1, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(0, mech.getVelocityRad_S().getAsDouble(), kDelta);
        // all the way there now
        assertEquals(1, sensor.getPositionRad().getAsDouble(), kDelta);
    }

    @Test
    void testSimpleSetpoint() {
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 100);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1, () -> 0);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        UnprofiledOutboardAngularPositionServo servo = new UnprofiledOutboardAngularPositionServo(
                log, mech);

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), kDelta);
        assertEquals(0, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(0, sensor.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, mech.getVelocityRad_S().getAsDouble(), kDelta);

        servo.periodic();
        servo.setPositionSetpoint(new Setpoints1d(new Control100(1, 0), new Control100(1, 0)), 0);
        stepTime();

        // move 0 to 1 in 0.02 => v = 50
        assertEquals(50, motor.getVelocityRad_S(), kDelta);
        assertEquals(1, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(50, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(50, mech.getVelocityRad_S().getAsDouble(), kDelta);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(0.5, sensor.getPositionRad().getAsDouble(), kDelta);

        servo.periodic();
        servo.setPositionSetpoint(new Setpoints1d(new Control100(1, 0), new Control100(1, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionSetpoint(new Setpoints1d(new Control100(1, 0), new Control100(1, 0)), 0);
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), kDelta);
        assertEquals(1, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(0, mech.getVelocityRad_S().getAsDouble(), kDelta);
        // all the way there now
        assertEquals(1, sensor.getPositionRad().getAsDouble(), kDelta);
    }
}
