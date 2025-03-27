package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

public class OnboardAngularPositionServoTest {
    // note ridiculously precise delta
    private static final double kDelta = 1e-9;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final boolean kActuallyPrint = false;

    @Test
    void testOnboard() {
        final MockBareMotor turningMotor = new MockBareMotor(Feedforward100.makeSimple());
        final RotaryMechanism mech = new SimpleRotaryMechanism(
                logger,
                turningMotor,
                new MockIncrementalBareEncoder(),
                1);
        final MockRotaryPositionSensor turningEncoder = new MockRotaryPositionSensor();
        final Feedback100 turningFeedback2 = new PIDFeedback(
                logger, 1, 0, 0, false, 0.05, 1);
        final Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        ProfiledController controller = new IncrementalProfiledController(
                logger,
                profile,
                turningFeedback2,
                MathUtil::angleModulus,
                0.05,
                0.05);
        final AngularPositionServo servo = new OnboardAngularPositionServo(
                logger,
                mech,
                turningEncoder,
                controller);
        servo.reset();
        // spin for 1 s
        for (int i = 0; i < 50; ++i) {
            servo.setPosition(1, 0);
            if (kActuallyPrint)
                Util.printf("i: %d position: %5.3f\n", i, turningMotor.position);
            // lets say we're on the profile.
            turningEncoder.angle = servo.getSetpoint().x();
            turningEncoder.rate = servo.getSetpoint().v();
        }
        assertEquals(0, turningMotor.output, 0.001);
        assertEquals(0.5, servo.getSetpoint().x(), kDelta);
        assertEquals(1.0, servo.getSetpoint().v(), kDelta);
        assertEquals(0.5, turningEncoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(1.000, turningMotor.velocity, kDelta);
    }

}
