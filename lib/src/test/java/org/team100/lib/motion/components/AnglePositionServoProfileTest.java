package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.math.MathUtil;

class AnglePositionServoProfileTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private final MockBareMotor motor;
    private final MockRotaryPositionSensor encoder;
    private final Feedback100 feedback2;
    private final AngularPositionServo servo;
    // for calculating the trapezoidal integral
    double previousMotorSpeed = 0;

    public AnglePositionServoProfileTest() {
        motor = new MockBareMotor();
        final RotaryMechanism mech = new SimpleRotaryMechanism(
                logger,
                motor,
                new MockIncrementalBareEncoder(),
                1);
        encoder = new MockRotaryPositionSensor();
        feedback2 = new PIDFeedback(logger, 1, 0, 0, true, 0.05, 1);

        final Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        ProfiledController controller = new ProfiledController(
                profile,
                feedback2,
                MathUtil::angleModulus,
                0.05,
                0.05);
        servo = new OnboardAngularPositionServo(
                logger,
                mech,
                encoder,
                controller);
        servo.reset();
    }

    @Test
    void testProfile() {
        verify(0.1, 0.005, 0.1);
        verify(0.2, 0.020, 0.2);
        verify(0.3, 0.045, 0.3);
        verify(0.4, 0.080, 0.4);
        verify(0.5, 0.125, 0.5);
        verify(0.6, 0.180, 0.6);
        verify(0.7, 0.245, 0.7);
        verify(0.8, 0.320, 0.8);
        verify(0.9, 0.405, 0.9);
        verify(1.0, 0.500, 1.0);
        verify(0.9, 0.595, 0.9);
        verify(0.8, 0.680, 0.8);
        verify(0.7, 0.755, 0.7);
        verify(0.6, 0.820, 0.6);
        verify(0.5, 0.875, 0.5);
        verify(0.4, 0.920, 0.4);
        verify(0.3, 0.955, 0.3);
        verify(0.2, 0.980, 0.2);
        verify(0.1, 0.995, 0.1);
        verify(0.0, 1.000, 0.0);
    }

    private void verify(
            double motorVelocity,
            double setpointPosition,
            double setpointVelocity) {
        // spin for 100ms
        for (int i = 0; i < 5; ++i) {
            // observe the current instant and set the output for the next step
            servo.setPosition(1, 0);
            // trapezoid integral over the step
            encoder.angle += 0.5 * (motor.velocity + previousMotorSpeed) * 0.02;
            previousMotorSpeed = motor.velocity;
        }
        assertEquals(motorVelocity, motor.velocity, kDelta);
        assertEquals(setpointPosition, servo.getSetpoint().x(), kDelta);
        assertEquals(setpointVelocity, servo.getSetpoint().v(), kDelta);
    }
}
