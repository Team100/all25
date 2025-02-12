package org.team100.lib.reference;

import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.util.Memo;

/** Produces references based on profiles. */
public class ProfileReference implements SwerveReference {
    /**
     * Putting these in the same class allows us to refresh them both atomically.
     */
    private record References(SwerveModel m_current, SwerveModel m_next) {
    }

    private final HolonomicProfile m_profile;
    private final Memo.CotemporalCache<References> m_references;

    private SwerveModel m_goal;

    /**
     * The most-recently calculated "next" reference, which will be a future
     * "current" reference.
     */
    private SwerveModel m_next;

    public ProfileReference(HolonomicProfile profile) {
        m_profile = profile;
        m_references = Memo.of(() -> refresh(m_next));
    }

    private References refresh(SwerveModel newCurrent) {
        m_next = makeNext(newCurrent);
        return new References(newCurrent, m_next);
    }

    public void setGoal(SwerveModel goal) {
        m_goal = goal;
    }

    /** Immediately overwrite the references. */
    @Override
    public void initialize(SwerveModel measurement) {
        m_profile.solve(measurement, m_goal);
        m_references.update(refresh(measurement));
    }

    @Override
    public SwerveModel current() {
        return m_references.get().m_current;
    }

    @Override
    public SwerveModel next() {
        return m_references.get().m_next;
    }

    private SwerveModel makeNext(SwerveModel current) {
        if (m_goal == null)
            return current;
        return m_profile.calculate(current, m_goal).model();
    }

}
