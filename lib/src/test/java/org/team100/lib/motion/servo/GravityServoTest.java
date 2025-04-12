package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleUnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.MockProfiledController;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.ProfiledController.Result;
import org.team100.lib.encoder.MockRotaryPositionSensor;
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
import org.team100.lib.state.Control100;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.MathUtil;

class GravityServoTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSetPosition() {
        Feedback100 pivotFeedback = new PIDFeedback(
                logger, 4.5, 0.0, 0.000, false, 0.05, 1);
        Profile100 profile = new TrapezoidProfile100(8, 8, 0.001);
        // motor speed is rad/s
        SimulatedBareMotor simMotor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, simMotor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                logger, encoder, 165, () -> 0);
        RotaryMechanism simMech = new RotaryMechanism(
                logger, simMotor, sensor, 165, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        ProfiledController controller = new IncrementalProfiledController(
                logger,
                profile,
                pivotFeedback,
                MathUtil::angleModulus,
                0.05,
                0.05);
        AngularPositionServo servo = new OnboardAngularPositionServo(
                logger, simMech, controller);
        servo.reset();

        Gravity gravity = new Gravity(logger, 5, 0);
        Spring spring = new Spring(logger);
        DoubleUnaryOperator torquefn = (x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x);
        Torque tt = new Torque(torquefn);
        // start at zero
        assertEquals(0, servo.getPosition().getAsDouble(), kDelta);
        // one second
        for (int i = 0; i < 70; ++i) {
            // double torque = g.torque();
            double torque = tt.torque(servo.getPosition());
            servo.setPosition(1, torque);
            stepTime();
        }
        assertEquals(1, servo.getPosition().getAsDouble(), 1e-5);
    }

    /** For refactoring the gravity servo */
    @Test
    void testGravity() {
        MockBareMotor motor = new MockBareMotor(Feedforward100.makeSimple());
        MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        MockProfiledController controller = new MockProfiledController();
        OnboardAngularPositionServo servo = new OnboardAngularPositionServo(logger, mech, controller);
        // these constants were used in Wrist2.
        // negative force means pull in
        Gravity gravity = new Gravity(logger, 9.0, -0.451230);
        Spring spring = new Spring(logger);
        DoubleUnaryOperator torquefn = (x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x);
        Torque tt = new Torque(torquefn);

        // the controller never does anything, so the only output should be the
        // gravity/spring feedforward torque.
        controller.result = new Result(new Control100(), 0);

        sensor.angle = -0.1;
        assertEquals(4.714, gravity.applyAsDouble(sensor.angle), kDelta);
        assertEquals(-9.888, spring.applyAsDouble(sensor.angle), kDelta);
        double torque = tt.torque(servo.getPosition());
        assertEquals(-5.175, torque, kDelta);
        servo.setPosition(0, torque);
        assertEquals(0, motor.velocity, kDelta);
        assertEquals(0, motor.accel, kDelta);
        assertEquals(-5.175, motor.torque, kDelta);

        sensor.angle = 0;
        assertEquals(3.925, gravity.applyAsDouble(sensor.angle), kDelta);
        assertEquals(-9.050, spring.applyAsDouble(sensor.angle), kDelta);
        torque = tt.torque(servo.getPosition());
        assertEquals(-5.125, torque, kDelta);
        servo.setPosition(0, torque);
        assertEquals(0, motor.velocity, kDelta);
        assertEquals(0, motor.accel, kDelta);
        assertEquals(-5.125, motor.torque, kDelta);

        sensor.angle = 0.5;
        assertEquals(-0.439, gravity.applyAsDouble(sensor.angle), kDelta);
        assertEquals(-5.631, spring.applyAsDouble(sensor.angle), kDelta);
        torque = tt.torque(servo.getPosition());
        assertEquals(-6.069, torque, kDelta);
        servo.setPosition(0, torque);
        assertEquals(0, motor.velocity, kDelta);
        assertEquals(0, motor.accel, kDelta);
        assertEquals(-6.069, motor.torque, kDelta);

        sensor.angle = 1.0;
        assertEquals(-4.695, gravity.applyAsDouble(sensor.angle), kDelta);
        assertEquals(-2.806, spring.applyAsDouble(sensor.angle), kDelta);
        torque = tt.torque(servo.getPosition());
        assertEquals(-7.500, torque, kDelta);
        servo.setPosition(0, torque);
        assertEquals(0, motor.velocity, kDelta);
        assertEquals(0, motor.accel, kDelta);
        assertEquals(-7.500, motor.torque, kDelta);

        sensor.angle = 1.5;
        assertEquals(-7.801, gravity.applyAsDouble(sensor.angle), kDelta);
        assertEquals(-1.576, spring.applyAsDouble(sensor.angle), kDelta);
        torque = tt.torque(servo.getPosition());
        assertEquals(-9.377, torque, kDelta);
        servo.setPosition(0, torque);
        assertEquals(0, motor.velocity, kDelta);
        assertEquals(0, motor.accel, kDelta);
        assertEquals(-9.377, motor.torque, kDelta);

        sensor.angle = 2.0;
        assertEquals(-8.998, gravity.applyAsDouble(sensor.angle), kDelta);
        assertEquals(-1.000, spring.applyAsDouble(sensor.angle), kDelta);
        torque = tt.torque(servo.getPosition());
        assertEquals(-9.998, torque, kDelta);
        servo.setPosition(0, torque);
        assertEquals(0, motor.velocity, kDelta);
        assertEquals(0, motor.accel, kDelta);
        assertEquals(-9.998, motor.torque, kDelta);

        sensor.angle = 2.5;
        assertEquals(-7.991, gravity.applyAsDouble(sensor.angle), kDelta);
        assertEquals(-1.0, spring.applyAsDouble(sensor.angle), kDelta);
        torque = tt.torque(servo.getPosition());
        assertEquals(-8.991, torque, kDelta);
        servo.setPosition(0, torque);
        assertEquals(0, motor.velocity, kDelta);
        assertEquals(0, motor.accel, kDelta);
        assertEquals(-8.991, motor.torque, kDelta);

        sensor.angle = 3.0;
        assertEquals(-5.028, gravity.applyAsDouble(sensor.angle), kDelta);
        assertEquals(-1.0, spring.applyAsDouble(sensor.angle), kDelta);
        torque = tt.torque(servo.getPosition());
        assertEquals(-6.028, torque, kDelta);
        servo.setPosition(0, torque);
        assertEquals(0, motor.velocity, kDelta);
        assertEquals(0, motor.accel, kDelta);
        assertEquals(-6.028, motor.torque, kDelta);
    }

}
