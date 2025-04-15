package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.testing.Timeless;

class AnglePositionServoProfileTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private final MockBareMotor motor;
    private final MockRotaryPositionSensor sensor;
    private final Feedback100 feedback2;
    private final IncrementalProfileReference1d ref;
    private final OnboardAngularPositionServo servo;
    // for calculating the trapezoidal integral
    double previousMotorSpeed = 0;

    public AnglePositionServoProfileTest() {
        motor = new MockBareMotor(Feedforward100.makeSimple());
        sensor = new MockRotaryPositionSensor();
        RotaryMechanism mech = new RotaryMechanism(
                logger,
                motor,
                sensor,
                1,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        feedback2 = new PIDFeedback(logger, 1, 0, 0, true, 0.05, 1);
        Profile100 profile = new TrapezoidProfile100(1, 1, 0.05);
        ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        servo = new OnboardAngularPositionServo(logger, mech, ref, feedback2);
        servo.reset();
    }

    @Test
    void testProfile() {
        // the profile pays attention to time, so this needs to be in the test method.
        // ref.init(new Model100());
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
            servo.setPositionProfiled(1, 0);
            stepTime();
            // trapezoid integral over the step
            sensor.angle += 0.5 * (motor.velocity + previousMotorSpeed) * 0.02;
            previousMotorSpeed = motor.velocity;
        }
        assertEquals(motorVelocity, motor.velocity, kDelta, "velocity");
        assertEquals(setpointPosition, servo.m_setpoint.x(), kDelta, "setpoint position");
        assertEquals(setpointVelocity, servo.m_setpoint.v(), kDelta, "setpoint velocity");
    }
}
