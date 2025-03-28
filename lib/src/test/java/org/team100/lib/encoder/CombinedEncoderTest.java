package org.team100.lib.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.testing.Timeless;

class CombinedEncoderTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testZeroing() {
        MockBareMotor motor = new MockBareMotor(Feedforward100.makeSimple());

        // this is the "correct" value
        MockRotaryPositionSensor e1 = new MockRotaryPositionSensor();
        e1.angle = 1;

        // this value is the "incorrect" value, should be overwritten by the combined
        // encoder constructor.
        MockIncrementalBareEncoder e2 = new MockIncrementalBareEncoder();
        e2.position = 0;

        RotaryMechanism m = new SimpleRotaryMechanism(logger, motor, e2, 1.0);
        CombinedEncoder c = new CombinedEncoder(logger, e1, m);// , true);
        c.sync();
        // the combined encoder reads the correct value
        assertEquals(1.0, c.getPositionRad().getAsDouble(), kDelta);

        // and the secondary encoder has been "fixed"
        assertEquals(1.0, e2.position, kDelta);
    }

    double lash = 0;

    @Test
    void testError() {
        // turn on the experiment we're testing
        Experiments.instance.testOverride(Experiment.LashCorrection, true);

        SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);

        assertEquals(0, encoder.getPositionRad().getAsDouble(), kDelta);

        SimpleRotaryMechanism mech = new SimpleRotaryMechanism(logger, motor, encoder, 1.0);

        // lash is added to the motor position to get this
        SimulatedRotaryPositionSensor e1 = new SimulatedRotaryPositionSensor(logger, mech, () -> lash);
        assertEquals(0, e1.getPositionRad().getAsDouble(), kDelta);

        CombinedEncoder c = new CombinedEncoder(logger, e1, mech);
        c.sync();

        assertEquals(0, c.getError(), kDelta);

        // actual drifts a bit: motor hasn't moved but actual is 0.5
        lash = 0.5;
        assertEquals(0, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0.5, e1.getPositionRad().getAsDouble(), kDelta);

        // combined encoder notices
        assertEquals(0.5, c.getError(), kDelta);

        // if we try to fix it with the same position ...
        stepTime();
        mech.setPosition(0, 0, 0, 0);
        stepTime();

        // nothing happens, motor is in the same place.
        assertEquals(0, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0.5, e1.getPositionRad().getAsDouble(), kDelta);

        // servo subtracts the error
        mech.setPosition(-0.5, 0, 0, 0);
        stepTime();
        assertEquals(-0.5, encoder.getPositionRad().getAsDouble(), kDelta);
        assertEquals(0.25, e1.getPositionRad().getAsDouble(), kDelta);

        // twice because of how the sim works
        mech.setPosition(-0.5, 0, 0, 0);
        stepTime();

        // now it's correct, motor is at -0.5 where we told it to go
        assertEquals(-0.5, encoder.getPositionRad().getAsDouble(), kDelta);
        // and the sensor reads 0
        assertEquals(0, e1.getPositionRad().getAsDouble(), kDelta);

        // the lash is the same unchanged
        assertEquals(0.5, c.getError(), kDelta);

    }
}
