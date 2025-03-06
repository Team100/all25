package org.team100.lib.motion.components;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;


class LinearVelocityServoTest {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        MockBareMotor driveMotor = new MockBareMotor(Feedforward100.makeSimple());
        MockIncrementalBareEncoder driveEncoder = new MockIncrementalBareEncoder();
        SimpleLinearMechanism mech = new SimpleLinearMechanism(driveMotor, driveEncoder, 1, 1);
        OutboardLinearVelocityServo servo = new OutboardLinearVelocityServo(
                logger,
                mech);
        // 0.5 m/s
        servo.setVelocityM_S(0.5);
        // wheel radius is 0.5 m, so drive speed is 1 m/s
        assertEquals(1.0, driveMotor.velocity, 0.001);
    }
}
