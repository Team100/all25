package org.team100.lib.reference.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.r1.TrapezoidProfileWPI;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;

public class IncrementalProfileReferenceR1Test implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        TrapezoidProfileWPI p = new TrapezoidProfileWPI(2, 6);
        Model100 goal = new Model100(1, 0);
        ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(log, () -> p, 0.05, 0.05);
        ref.setGoal(goal);
        Model100 measurement = new Model100();
        ref.init(measurement);

        // initial current setpoint is the measurement.
        SetpointsR1 s = ref.get();
        assertEquals(0, s.current().x(), DELTA);
        assertEquals(0, s.current().v(), DELTA);
        assertEquals(0, s.current().a(), DELTA);
        assertEquals(0.0012, s.next().x(), DELTA);
        assertEquals(0.120, s.next().v(), DELTA);
        assertEquals(6.000, s.next().a(), DELTA);

        // if time does not pass, nothing changes.
        s = ref.get();
        assertEquals(0, s.current().x(), DELTA);
        assertEquals(0, s.current().v(), DELTA);
        assertEquals(0, s.current().a(), DELTA);
        assertEquals(0.0012, s.next().x(), DELTA);
        assertEquals(0.120, s.next().v(), DELTA);
        assertEquals(6.000, s.next().a(), DELTA);

        stepTime();

        // now the setpoint has advanced

        s = ref.get();
        assertEquals(0.0012, s.current().x(), DELTA);
        assertEquals(0.120, s.current().v(), DELTA);
        assertEquals(6.000, s.current().a(), DELTA);
        assertEquals(0.0048, s.next().x(), DELTA);
        assertEquals(0.240, s.next().v(), DELTA);
        assertEquals(6.000, s.next().a(), DELTA);

    }
}
