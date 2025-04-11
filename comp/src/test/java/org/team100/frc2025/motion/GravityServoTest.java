package org.team100.frc2025.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2025.Timeless2025;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;

import edu.wpi.first.math.MathUtil;

class GravityServoTest implements Timeless2025 {
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
        RotaryMechanism simMech = new RotaryMechanism(
                logger, simMotor, encoder, 165, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        SimulatedRotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                logger,
                simMech,
                () -> 0);

        ProfiledController controller = new IncrementalProfiledController(
                logger,
                profile,
                pivotFeedback,
                MathUtil::angleModulus,
                0.05,
                0.05);
        AngularPositionServo servo = new OnboardAngularPositionServo(
                logger,
                simMech,
                simEncoder,
                controller);
        servo.reset();

        GravityServoInterface g = new OutboardGravityServo(logger,
                servo, 5.0, 0.0);
        // start at zero
        assertEquals(0, g.getPositionRad().getAsDouble(), kDelta);
        // one second
        for (int i = 0; i < 70; ++i) {
            g.setPosition(1);
            stepTime();
        }
        assertEquals(1, g.getPositionRad().getAsDouble(), 1e-5);
    }
}
