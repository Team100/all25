package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.ZeroFeedback;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.MockIncrementalBareEncoder;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.state.Control100;

public class OutboardAngularPositionServoWithoutWrapTest {
    private static final double kDelta = 0.001;
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    // @Test
    void testWristMock() {
        // this is the wrist scenario

        Feedforward100 wristFF = Feedforward100.makeKraken6WristWithLowerKd();

        MockBareMotor wristMotor = new MockBareMotor(wristFF);

        MockIncrementalBareEncoder internalWristEncoder = new MockIncrementalBareEncoder();

        RotaryMechanism wristMech = new SimpleRotaryMechanism(log, wristMotor, internalWristEncoder, 9);

        MockRotaryPositionSensor encoder = new MockRotaryPositionSensor();

        CombinedEncoder combinedEncoder = new CombinedEncoder(log, encoder, wristMech);// , false);

        TrapezoidProfile100 wristProfile = new TrapezoidProfile100(50, 50, 0.05);
        ZeroFeedback feedback = new ZeroFeedback(x -> x, 0.05, 0.05);
        ProfiledController controller = new IncrementalProfiledController(
                log, wristProfile, feedback, x -> x, 0.05, 0.05);

        // the servo does not actually use the encoder measurement for anything; it
        // passes
        // the position command to the motor, and uses the profile feedforward.
        AngularPositionServo wristServoWithoutGravity = new OutboardAngularPositionServoWithoutWrap(
                log, wristMech,
                combinedEncoder, controller);

        OutboardGravityServo wristServo = new OutboardGravityServo(log, wristServoWithoutGravity, 4, 0);

        Control100 control = new Control100(1.36, 0, 0);

        // start motionless at the setpoint
        assertEquals(0, wristMotor.position, kDelta);
        assertEquals(0, wristMotor.velocity, kDelta);
        assertEquals(0, wristMotor.accel, kDelta);
        assertEquals(0, wristMotor.torque, kDelta);
        assertEquals(0, wristMotor.ffVolts, kDelta);
        assertEquals(0, wristServoWithoutGravity.getSetpoint().x(), kDelta);

        wristServo.setState(control);

        // accel 50 * 0.02s -> v = 1, x = vdt/2 -> 0.01, * 9 gearing
        assertEquals(0.09, wristMotor.position, kDelta);
        // v =1 * 9
        assertEquals(9, wristMotor.velocity, kDelta);
        // accel 50 * 9
        assertEquals(450, wristMotor.accel, kDelta);
        // gravity ~ sin(x) = 0
        assertEquals(0, wristMotor.torque, kDelta);
        assertEquals(0.230, wristMotor.frictionFFVolts, kDelta);
        assertEquals(0.201, wristMotor.velocityFFVolts, kDelta);
        assertEquals(0, wristMotor.torqueFFVolts, kDelta);
        assertEquals(0.931, wristMotor.accelFFVolts, kDelta);
        assertEquals(1.361, wristMotor.ffVolts, kDelta);
        // position is at the setpoint since it's a mock motor
        assertEquals(0.01, wristServoWithoutGravity.getSetpoint().x(), kDelta);

        // step a few times
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);

        assertEquals(3.24, wristMotor.position, kDelta);
        assertEquals(54, wristMotor.velocity, kDelta);
        // same accel
        assertEquals(450, wristMotor.accel, kDelta);
        // why is this negative?
        assertEquals(-0.063, wristMotor.torque, kDelta);
        assertEquals(0.230, wristMotor.frictionFFVolts, kDelta);
        assertEquals(1.203, wristMotor.velocityFFVolts, kDelta);
        // ??
        assertEquals(-0.316, wristMotor.torqueFFVolts, kDelta);
        assertEquals(0.931, wristMotor.accelFFVolts, kDelta);
        assertEquals(2.048, wristMotor.ffVolts, kDelta);
        assertEquals(0.36, wristServoWithoutGravity.getSetpoint().x(), kDelta);

        // a few more times
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);

        assertEquals(9.525, wristMotor.position, kDelta);
        assertEquals(49.432, wristMotor.velocity, kDelta);
        // decel now
        assertEquals(-450, wristMotor.accel, kDelta);
        // ???
        assertEquals(-0.227, wristMotor.torque, kDelta);
        assertEquals(0.230, wristMotor.frictionFFVolts, kDelta);
        assertEquals(1.101, wristMotor.velocityFFVolts, kDelta);
        // ???
        assertEquals(-1.135, wristMotor.torqueFFVolts, kDelta);
        // braking => use kD value, which is lower
        assertEquals(-0.358, wristMotor.accelFFVolts, kDelta);
        assertEquals(-0.162, wristMotor.ffVolts, kDelta);
        assertEquals(1.058, wristServoWithoutGravity.getSetpoint().x(), kDelta);

        // almost done
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);

        assertEquals(12.218, wristMotor.position, kDelta);
        // almost stopped
        assertEquals(4.432, wristMotor.velocity, kDelta);
        // still decel
        assertEquals(-450, wristMotor.accel, kDelta);
        // ???
        assertEquals(-0.308, wristMotor.torque, kDelta);
        assertEquals(0.230, wristMotor.frictionFFVolts, kDelta);
        assertEquals(0.099, wristMotor.velocityFFVolts, kDelta);
        assertEquals(-1.538, wristMotor.torqueFFVolts, kDelta);
        assertEquals(-0.358, wristMotor.accelFFVolts, kDelta);
        assertEquals(-1.567, wristMotor.ffVolts, kDelta);

        // almost done
        assertEquals(1.357, wristServoWithoutGravity.getSetpoint().x(), kDelta);

        // done
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);
        internalWristEncoder.position = wristMotor.position;
        wristServo.setState(control);

        assertEquals(12.24, wristMotor.position, kDelta);
        assertEquals(0, wristMotor.velocity, kDelta);
        assertEquals(0, wristMotor.accel, kDelta);
        assertEquals(-0.312, wristMotor.torque, kDelta);
        assertEquals(0, wristMotor.frictionFFVolts, kDelta);
        assertEquals(0, wristMotor.velocityFFVolts, kDelta);
        assertEquals(-1.558, wristMotor.torqueFFVolts, kDelta);
        assertEquals(0, wristMotor.accelFFVolts, kDelta);
        assertEquals(-1.558, wristMotor.ffVolts, kDelta);
        assertEquals(1.36, wristServoWithoutGravity.getSetpoint().x(), kDelta);
    }
}
