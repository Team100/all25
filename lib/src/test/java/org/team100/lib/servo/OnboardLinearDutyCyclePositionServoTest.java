package org.team100.lib.servo;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.controller.r1.FullStateFeedback;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.testing.Timeless;

public class OnboardLinearDutyCyclePositionServoTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test1() {

        SimulatedBareMotor driveMotor = new SimulatedBareMotor(logger, 600);
        IncrementalBareEncoder driveEncoder = driveMotor.encoder();
        LinearMechanism mech = new LinearMechanism(logger,
                driveMotor, driveEncoder, 1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        IncrementalProfile profile = new TrapezoidIncrementalProfile(logger, 2, 1, 0.01);
        IncrementalProfileReferenceR1 ref = new IncrementalProfileReferenceR1(logger, () -> profile, 0.05, 0.05);

        final double k1 = 1.0;
        final double k2 = 0.01;
        Feedback100 feedback = new FullStateFeedback(logger, k1, k2, false, 1, 1);

        OnboardLinearDutyCyclePositionServo s = new OnboardLinearDutyCyclePositionServo(
                logger, mech, ref, feedback, 0.1);
        s.reset();
        for (double t = 0; t < 3; t += 0.02) {
            s.setPositionProfiled(1, 0);
            stepTime();
            if (DEBUG)
                System.out.printf("%f, %f, %f, %f, %f\n",
                        t,
                        driveMotor.getVelocityRad_S(),
                        driveEncoder.getVelocityRad_S(),
                        driveEncoder.getUnwrappedPositionRad(),
                        mech.getPositionM());
        }

    }

}
