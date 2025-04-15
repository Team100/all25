package org.team100.lib.reference;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;

public class TimedProfileReference1dTest implements Timeless {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        JerkLimitedProfile100 p = new JerkLimitedProfile100(2, 6, 25, false);
        Model100 goal = new Model100(1, 0);
        ProfileReference1d ref = new TimedProfileReference1d(p);
        ref.setGoal(goal);
        Model100 measurement = new Model100();
        ref.init(measurement);

        // initial current setpoint is the measurement.
        Setpoints1d s = ref.get();
        assertEquals(0, s.current().x(), kDelta);
        assertEquals(0, s.current().v(), kDelta);
        assertEquals(0, s.current().a(), kDelta);
        assertEquals(0.00003, s.next().x(), kDelta);
        assertEquals(0.005, s.next().v(), kDelta);
        assertEquals(0.500, s.next().a(), kDelta);

        // if time does not pass, nothing changes.
        s = ref.get();
        assertEquals(0, s.current().x(), kDelta);
        assertEquals(0, s.current().v(), kDelta);
        assertEquals(0, s.current().a(), kDelta);
        assertEquals(0.00003, s.next().x(), kDelta);
        assertEquals(0.005, s.next().v(), kDelta);
        assertEquals(0.500, s.next().a(), kDelta);

        stepTime();

        // now the setpoint has advanced
        s = ref.get();
        assertEquals(0, s.current().x(), kDelta);
        assertEquals(0.005, s.current().v(), kDelta);
        assertEquals(0.500, s.current().a(), kDelta);
        assertEquals(0.00026, s.next().x(), kDelta);
        assertEquals(0.020, s.next().v(), kDelta);
        assertEquals(1.000, s.next().a(), kDelta);

    }

}
