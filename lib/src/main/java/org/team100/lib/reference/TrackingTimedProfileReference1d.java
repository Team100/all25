package org.team100.lib.reference;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Takt;

/** Update goal on init. */
public class TrackingTimedProfileReference1d {

    private final TimedProfile m_profile;
    private double m_startTimeS;

    public TrackingTimedProfileReference1d(TimedProfile profile) {
        m_profile = profile;
    }

    public void init(Model100 measurement, Model100 goal) {
        m_startTimeS = Takt.get();
        m_profile.init(measurement.control(), goal);
    }

    public Setpoints1d get() {
        double progress = progress();
        return new Setpoints1d(
                m_profile.sample(progress),
                m_profile.sample(progress + TimedRobot100.LOOP_PERIOD_S));
    }

    public double duration() {
        return m_profile.duration();
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

    public boolean profileDone() {
        return progress() >= duration();
    }
}
