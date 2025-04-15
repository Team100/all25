package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.ZeroFeedback;
import org.team100.lib.encoder.CombinedRotaryPositionSensor;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

public class OutboardAngularPositionServoTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final boolean kActuallyPrint = false;

    @Test
    void testOutboard() {
        final MockBareMotor motor = new MockBareMotor(Feedforward100.makeSimple());
        final MockIncrementalBareEncoder encoder = new MockIncrementalBareEncoder();
        final MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();

        final ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(encoder, 1);
        final CombinedRotaryPositionSensor combinedEncoder = new CombinedRotaryPositionSensor(
                logger, sensor, proxy);

        final RotaryMechanism mech = new RotaryMechanism(
                logger, motor, combinedEncoder, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        final Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        Model100 goal = new Model100(1, 0);
        Model100 measurement = new Model100();
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, goal, 0.01, 0.01);
        ref.init(measurement);
        // final ZeroFeedback feedback = new ZeroFeedback(x -> x, 0.01, 0.01);
        // final ProfiledController controller = new IncrementalProfiledController(
        //         logger, ref, feedback, x -> x, 0.01, 0.01);
        final OutboardAngularPositionServo servo = new OutboardAngularPositionServo(
                logger, mech);
        servo.reset();
        // it moves slowly
        servo.setPositionSetpoint(ref.get(), 0);
        stepTime();

        assertEquals(2e-4, motor.position, 1e-4);
        servo.setPositionSetpoint(ref.get(), 0);
        stepTime();

        // assertEquals(8e-4, motor.position, 1e-4);
        servo.setPositionSetpoint(ref.get(), 0);
        stepTime();

        assertEquals(0.002, motor.position, kDelta);
        for (int i = 0; i < 100; ++i) {
            // run it for awhile
            servo.setPositionSetpoint(ref.get(), 0);
            stepTime();
            if (kActuallyPrint)
                Util.printf("i: %d position: %5.3f\n", i, motor.position);
        }
        assertEquals(1, motor.position, kDelta);
    }
}
