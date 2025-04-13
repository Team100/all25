package org.team100.lib.reference;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Takt;

/**
 * To extract current and next references from the timed profile
 * controller.
 * 
 * Contains the goal, since our goals never change.
 */
public class TimedProfileReference1d {

    private final TimedProfile m_profile;
    private final Model100 m_goal;
    private double m_startTimeS;

    public TimedProfileReference1d(TimedProfile profile, Model100 goal) {
        m_profile = profile;
        m_goal = goal;
    }

    public void init(Model100 measurement) {
        m_startTimeS = Takt.get();
        m_profile.init(measurement.control(), m_goal);
    }

    public Setpoints1d get() {
        double progress = progress();
        return new Setpoints1d(
                m_profile.sample(progress),
                m_profile.sample(progress + TimedRobot100.LOOP_PERIOD_S));
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

}
