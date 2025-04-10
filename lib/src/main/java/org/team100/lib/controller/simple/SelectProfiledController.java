package org.team100.lib.controller.simple;

import static java.util.Map.entry;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.incremental.CurrentLimitedExponentialProfile;
import org.team100.lib.profile.incremental.ExponentialProfileWPI;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.profile.incremental.TrapezoidProfileWPI;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.profile.timed.SepticSplineProfile;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

/**
 * Allow on-the-fly selection of profiles using the proxy pattern.
 */
public class SelectProfiledController implements ProfiledController, Glassy {
    private static final boolean DEBUG = false;

    public enum ProfileChoice {
        TRAPEZOID_100,
        TRAPEZOID_WPI,
        EXPONENTIAL_WPI,
        CURRENT_LIMITED,
        JERK_LIMITED,
        SEPTIC
    }

    /** For initializing. */
    private final DoubleSupplier m_measurement;
    private final Map<ProfileChoice, ProfiledController> m_controllers;

    private ProfiledController m_selected;

    public SelectProfiledController(
            LoggerFactory parent,
            Feedback100 feedback,
            DoubleUnaryOperator mod,
            DoubleSupplier measurement,
            double vel,
            double accel,
            double stall,
            double jerk,
            double pTol,
            double vTol) {
        LoggerFactory child = parent.child(this);
        m_measurement = measurement;
        m_controllers = Map.ofEntries(
                entry(ProfileChoice.CURRENT_LIMITED,
                        new IncrementalProfiledController(child,
                                new CurrentLimitedExponentialProfile(vel, accel, stall),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.EXPONENTIAL_WPI,
                        new IncrementalProfiledController(child,
                                new ExponentialProfileWPI(vel, accel),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.JERK_LIMITED,
                        new TimedProfiledController(child,
                                new JerkLimitedProfile100(vel, accel, jerk, false),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.SEPTIC,
                        new TimedProfiledController(child,
                                new SepticSplineProfile(4.5, 10),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.TRAPEZOID_100,
                        new IncrementalProfiledController(child,
                                new TrapezoidProfile100(vel, accel, pTol),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.TRAPEZOID_WPI,
                        new IncrementalProfiledController(child,
                                new TrapezoidProfileWPI(vel, accel),
                                feedback, mod, pTol, vTol)));
        // temporary default
        setDelegate(ProfileChoice.TRAPEZOID_100);
    }

    public void setDelegate(ProfileChoice choice) {
        if (DEBUG)
            Util.printf("SelectProfiledController setDelegate %s\n", choice);
        m_selected = m_controllers.get(choice);
        // Must initialize when switching controllers.
        // For safety, use zero velocity.
        double measurement = m_measurement.getAsDouble();
        m_selected.init(new Model100(measurement, 0));
    }

    @Override
    public void init(Model100 measurement) {
        if (DEBUG)
            Util.printf("SelectProfiledController init\n");
        m_selected.init(measurement);
    }

    @Override
    public Result calculate(Model100 measurement, Model100 goal) {
        return m_selected.calculate(measurement, goal);
    }

    @Override
    public boolean profileDone() {
        return m_selected.profileDone();
    }

    @Override
    public Model100 getSetpoint() {
        return m_selected.getSetpoint();
    }

    @Override
    public boolean atSetpoint() {
        return m_selected.atSetpoint();
    }

    @Override
    public boolean atGoal(Model100 goal) {
        return m_selected.atGoal(goal);
    }

    @Override
    public void close() {
        for (ProfiledController c : m_controllers.values()) {
            c.close();
        }
    }
}
