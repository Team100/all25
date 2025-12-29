package org.team100.lib.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleUnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.controller.r1.ZeroFeedback;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.MockProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.sensor.position.absolute.MockRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;

class GravityServoTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSetPosition() {
        Feedback100 pivotFeedback = new PIDFeedback(
                logger, 4.5, 0.0, 0.000, false, 0.05, 1);
        IncrementalProfile profile = new TrapezoidIncrementalProfile(logger, 8, 8, 0.001);
        IncrementalProfileReferenceR1 ref = new IncrementalProfileReferenceR1(logger,() ->  profile, 0.05, 0.05);
        // motor speed is rad/s
        SimulatedBareMotor simMotor = new SimulatedBareMotor(logger, 600);
        IncrementalBareEncoder encoder = simMotor.encoder();
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                logger, encoder, 165);
        RotaryMechanism simMech = new RotaryMechanism(
                logger, simMotor, sensor, 165, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        AngularPositionServo servo = new OnboardAngularPositionServo(
                logger, simMech, ref, pivotFeedback);
        servo.reset();
        ref.init(new Model100(servo.getWrappedPositionRad(), 0));

        Gravity gravity = new Gravity(logger, 5, 0);
        Spring spring = new Spring(logger);
        DoubleUnaryOperator torquefn = (x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x);
        Torque tt = new Torque(torquefn);
        // start at zero
        assertEquals(0, servo.getWrappedPositionRad(), DELTA);
        // one second
        for (int i = 0; i < 70; ++i) {
            double torque = tt.torque(servo.getWrappedPositionRad());
            servo.setPositionProfiled(1, torque);
            stepTime();
        }
        assertEquals(1, servo.getWrappedPositionRad(), 1e-5);
    }

    /** For refactoring the gravity servo */
    @Test
    void testGravity() {
        MockBareMotor motor = new MockBareMotor(Feedforward100.makeSimple(logger));
        MockRotaryPositionSensor sensor = new MockRotaryPositionSensor();
        RotaryMechanism mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        Feedback100 fb = new ZeroFeedback(false, 0.01, 0.01);
        ProfileReferenceR1 ref = new MockProfileReferenceR1();
        OnboardAngularPositionServo servo = new OnboardAngularPositionServo(logger, mech, ref, fb);
        // these constants were used in Wrist2.
        // negative force means pull in
        Gravity gravity = new Gravity(logger, 9.0, -0.451230);
        Spring spring = new Spring(logger);
        DoubleUnaryOperator torquefn = (x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x);
        Torque tt = new Torque(torquefn);

        // the controller never does anything, so the only output should be the
        // gravity/spring feedforward torque.

        sensor.angle = -0.1;
        assertEquals(4.714, gravity.applyAsDouble(sensor.angle), DELTA);
        assertEquals(-9.888, spring.applyAsDouble(sensor.angle), DELTA);
        double torque = tt.torque(servo.getWrappedPositionRad());
        assertEquals(-5.175, torque, DELTA);
        servo.setPositionDirect(0, 0, torque);
        assertEquals(0, motor.velocity, DELTA);
        assertEquals(0, motor.accel, DELTA);
        assertEquals(-5.175, motor.torque, DELTA);

        sensor.angle = 0;
        assertEquals(3.925, gravity.applyAsDouble(sensor.angle), DELTA);
        assertEquals(-9.050, spring.applyAsDouble(sensor.angle), DELTA);
        torque = tt.torque(servo.getWrappedPositionRad());
        assertEquals(-5.125, torque, DELTA);
        servo.setPositionDirect(0, 0, torque);
        assertEquals(0, motor.velocity, DELTA);
        assertEquals(0, motor.accel, DELTA);
        assertEquals(-5.125, motor.torque, DELTA);

        sensor.angle = 0.5;
        assertEquals(-0.439, gravity.applyAsDouble(sensor.angle), DELTA);
        assertEquals(-5.631, spring.applyAsDouble(sensor.angle), DELTA);
        torque = tt.torque(servo.getWrappedPositionRad());
        assertEquals(-6.069, torque, DELTA);
        servo.setPositionDirect(0, 0, torque);
        assertEquals(0, motor.velocity, DELTA);
        assertEquals(0, motor.accel, DELTA);
        assertEquals(-6.069, motor.torque, DELTA);

        sensor.angle = 1.0;
        assertEquals(-4.695, gravity.applyAsDouble(sensor.angle), DELTA);
        assertEquals(-2.806, spring.applyAsDouble(sensor.angle), DELTA);
        torque = tt.torque(servo.getWrappedPositionRad());
        assertEquals(-7.500, torque, DELTA);
        servo.setPositionDirect(0, 0, torque);
        assertEquals(0, motor.velocity, DELTA);
        assertEquals(0, motor.accel, DELTA);
        assertEquals(-7.500, motor.torque, DELTA);

        sensor.angle = 1.5;
        assertEquals(-7.801, gravity.applyAsDouble(sensor.angle), DELTA);
        assertEquals(-1.576, spring.applyAsDouble(sensor.angle), DELTA);
        torque = tt.torque(servo.getWrappedPositionRad());
        assertEquals(-9.377, torque, DELTA);
        servo.setPositionDirect(0, 0, torque);
        assertEquals(0, motor.velocity, DELTA);
        assertEquals(0, motor.accel, DELTA);
        assertEquals(-9.377, motor.torque, DELTA);

        sensor.angle = 2.0;
        assertEquals(-8.998, gravity.applyAsDouble(sensor.angle), DELTA);
        assertEquals(-1.000, spring.applyAsDouble(sensor.angle), DELTA);
        torque = tt.torque(servo.getWrappedPositionRad());
        assertEquals(-9.998, torque, DELTA);
        servo.setPositionDirect(0, 0, torque);
        assertEquals(0, motor.velocity, DELTA);
        assertEquals(0, motor.accel, DELTA);
        assertEquals(-9.998, motor.torque, DELTA);

        sensor.angle = 2.5;
        assertEquals(-7.991, gravity.applyAsDouble(sensor.angle), DELTA);
        assertEquals(-1.0, spring.applyAsDouble(sensor.angle), DELTA);
        torque = tt.torque(servo.getWrappedPositionRad());
        assertEquals(-8.991, torque, DELTA);
        servo.setPositionDirect(0, 0, torque);
        assertEquals(0, motor.velocity, DELTA);
        assertEquals(0, motor.accel, DELTA);
        assertEquals(-8.991, motor.torque, DELTA);

        sensor.angle = 3.0;
        assertEquals(-5.028, gravity.applyAsDouble(sensor.angle), DELTA);
        assertEquals(-1.0, spring.applyAsDouble(sensor.angle), DELTA);
        torque = tt.torque(servo.getWrappedPositionRad());
        assertEquals(-6.028, torque, DELTA);
        servo.setPositionDirect(0, 0, torque);
        assertEquals(0, motor.velocity, DELTA);
        assertEquals(0, motor.accel, DELTA);
        assertEquals(-6.028, motor.torque, DELTA);
    }

}
