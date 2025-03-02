package org.team100.lib.motion.servo;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.FullStateFeedback;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

public class OnboardLinearDutyCyclePositionServoTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test1() {

        SimulatedBareMotor driveMotor = new SimulatedBareMotor(logger, 100);
        SimulatedBareEncoder driveEncoder = new SimulatedBareEncoder(logger, driveMotor);
        SimpleLinearMechanism mech = new SimpleLinearMechanism(driveMotor, driveEncoder, 1, 1);

        Profile100 p = new TrapezoidProfile100(2, 1, 0.01);

        final double k1 = 1.0;
        final double k2 = 0.01;
        Feedback100 f = new FullStateFeedback(logger, k1, k2, x -> x, 1, 1);
        ProfiledController c = new ProfiledController(p, f, x -> x, 0.05, 0.05);

        OnboardLinearDutyCyclePositionServo s = new OnboardLinearDutyCyclePositionServo(logger, mech, c, 0.1);
        s.reset();
        for (double t = 0; t < 3; t += 0.02) {
            s.setPosition(1, 0);
            stepTime();
            if (DEBUG)
                Util.printf("%f, %f, %f, %f, %f\n",
                        t,
                        driveMotor.getVelocityRad_S(),
                        driveEncoder.getVelocityRad_S().getAsDouble(),
                        driveEncoder.getPositionRad().getAsDouble(),
                        mech.getPositionM().getAsDouble());
        }

    }

}
