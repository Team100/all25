package org.team100.lib.motion.servo;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.FullStateFeedback;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

public class OnboardLinearDutyCyclePositionServoTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test1() {

        SimulatedBareMotor driveMotor = new SimulatedBareMotor(logger, 100);
        SimulatedBareEncoder driveEncoder = new SimulatedBareEncoder(logger, driveMotor);
        LinearMechanism mech = new LinearMechanism(
                driveMotor, driveEncoder, 1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        Profile100 profile = new TrapezoidProfile100(2, 1, 0.01);
        Model100 goal = new Model100(1, 0);
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(profile, goal, 0.05, 0.05);
        ref.init(new Model100());

        final double k1 = 1.0;
        final double k2 = 0.01;
        Feedback100 feedback = new FullStateFeedback(logger, k1, k2, x -> x, 1, 1);
        // ProfiledController c = new IncrementalProfiledController(
        // logger, ref, f, x -> x, 0.05, 0.05);

        OnboardLinearDutyCyclePositionServo s = new OnboardLinearDutyCyclePositionServo(logger, mech, feedback, 0.1);
        s.reset();
        for (double t = 0; t < 3; t += 0.02) {
            Setpoints1d setpoints = ref.get();
            s.setPositionSetpoint(setpoints, 0);
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
