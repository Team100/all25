package org.team100.lib.reference;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.incremental.TrapezoidProfileWPI;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;

public class IncrementalProfileReference1dTest implements Timeless {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        TrapezoidProfileWPI p = new TrapezoidProfileWPI(2, 6);
        Model100 goal = new Model100(1, 0);
        IncrementalProfileReference1d ref = new IncrementalProfileReference1d(p, goal);
        Model100 measurement = new Model100();
        ref.init(measurement);

        // initial current setpoint is the measurement.
        Setpoints1d s = ref.get();
        assertEquals(0, s.current().x(), kDelta);
        assertEquals(0, s.current().v(), kDelta);
        assertEquals(0, s.current().a(), kDelta);
        assertEquals(0.0012, s.next().x(), kDelta);
        assertEquals(0.120, s.next().v(), kDelta);
        assertEquals(6.000, s.next().a(), kDelta);

        // if time does not pass, nothing changes.
        s = ref.get();
        assertEquals(0, s.current().x(), kDelta);
        assertEquals(0, s.current().v(), kDelta);
        assertEquals(0, s.current().a(), kDelta);
        assertEquals(0.0012, s.next().x(), kDelta);
        assertEquals(0.120, s.next().v(), kDelta);
        assertEquals(6.000, s.next().a(), kDelta);

        stepTime();

        // now the setpoint has advanced

        s = ref.get();
        assertEquals(0.0012, s.current().x(), kDelta);
        assertEquals(0.120, s.current().v(), kDelta);
        assertEquals(6.000, s.current().a(), kDelta);
        assertEquals(0.0048, s.next().x(), kDelta);
        assertEquals(0.240, s.next().v(), kDelta);
        assertEquals(6.000, s.next().a(), kDelta);


    }
}
