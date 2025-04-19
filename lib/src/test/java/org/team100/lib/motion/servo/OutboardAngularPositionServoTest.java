package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.encoder.CombinedRotaryPositionSensor;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.MockProfileReference1d;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

public class OutboardAngularPositionServoTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final boolean kActuallyPrint = false;

    @Test
    void testProfiled() {
        final MockBareMotor motor = new MockBareMotor(Feedforward100.makeSimple());
        final MockIncrementalBareEncoder encoder = new MockIncrementalBareEncoder();
        final MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();

        final ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(encoder, 1);
        final CombinedRotaryPositionSensor combinedEncoder = new CombinedRotaryPositionSensor(
                log, sensor, proxy);

        final RotaryMechanism mech = new RotaryMechanism(
                log, motor, combinedEncoder, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        final Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        final IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.01, 0.01);
        final OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);
        servo.reset();
        // it moves slowly
        servo.setPositionProfiled(1, 0);
        stepTime();

        assertEquals(2e-4, motor.position, 1e-4);
        servo.setPositionProfiled(1, 0);
        stepTime();

        // assertEquals(8e-4, motor.position, 1e-4);
        servo.setPositionProfiled(1, 0);
        stepTime();

        assertEquals(0.002, motor.position, kDelta);
        for (int i = 0; i < 100; ++i) {
            // run it for awhile
            servo.setPositionProfiled(1, 0);
            stepTime();
            if (kActuallyPrint)
                Util.printf("i: %d position: %5.3f\n", i, motor.position);
        }
        assertEquals(1, motor.position, kDelta);
    }

    @Test
    void testDirect() {
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 100);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        ProfileReference1d ref = new MockProfileReference1d();
        OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                log, mech, ref);

        servo.reset();
        servo.periodic();
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), kDelta);
        assertEquals(0, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(0, sensor.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, mech.getVelocityRad_S().getAsDouble(), kDelta);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(1, 0)), 0);
        stepTime();

        // move 0 to 1 in 0.02 => v = 50
        assertEquals(50, motor.getVelocityRad_S(), kDelta);
        assertEquals(1, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(50, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(50, mech.getVelocityRad_S().getAsDouble(), kDelta);
        // the sensor does trapezoid integration so it's halfway there after one cycle
        assertEquals(0.5, sensor.getPositionRad().getAsDouble(), kDelta);

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(1, 0)), 0);
        stepTime();

        servo.periodic();
        servo.setPositionDirect(new Setpoints1d(new Control100(0, 0), new Control100(1, 0)), 0);
        stepTime();

        assertEquals(0, motor.getVelocityRad_S(), kDelta);
        assertEquals(1, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0, encoder.getVelocityRad_S().getAsDouble(), kDelta);
        assertEquals(0, mech.getVelocityRad_S().getAsDouble(), kDelta);
        // all the way there now
        assertEquals(1, sensor.getPositionRad().getAsDouble(), kDelta);
    }
}
