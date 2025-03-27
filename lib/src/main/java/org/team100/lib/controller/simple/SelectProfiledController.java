package org.team100.lib.controller.simple;

import static java.util.Map.entry;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import org.team100.lib.async.Async;
import org.team100.lib.profile.CurrentLimitedExponentialProfile;
import org.team100.lib.profile.ExponentialProfileWPI;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.profile.TrapezoidProfileWPI;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.profile.timed.SepticSplineProfile;
import org.team100.lib.state.Model100;
import org.team100.lib.util.PolledEnumChooser;

/**
 * Allow on-the-fly selection of profiles using the proxy pattern.
 */
public class SelectProfiledController implements ProfiledController {
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
    private final PolledEnumChooser<ProfileChoice> m_chooser;
    private final Map<ProfileChoice, ProfiledController> m_controllers;

    private ProfiledController m_selected;

    public SelectProfiledController(
            String name,
            Async async,
            Feedback100 feedback,
            DoubleUnaryOperator mod,
            DoubleSupplier measurement,
            double vel,
            double accel,
            double stall,
            double jerk,
            double pTol,
            double vTol) {
        m_measurement = measurement;
        m_controllers = Map.ofEntries(
                entry(ProfileChoice.CURRENT_LIMITED,
                        new IncrementalProfiledController(
                                new CurrentLimitedExponentialProfile(vel, accel, stall),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.EXPONENTIAL_WPI,
                        new IncrementalProfiledController(
                                new ExponentialProfileWPI(vel, accel),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.JERK_LIMITED,
                        new TimedProfiledController(
                                new JerkLimitedProfile100(vel, accel, jerk),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.SEPTIC,
                        new TimedProfiledController(
                                new SepticSplineProfile(vel, accel),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.TRAPEZOID_100,
                        new IncrementalProfiledController(
                                new TrapezoidProfile100(vel, accel, pTol),
                                feedback, mod, pTol, vTol)),
                entry(ProfileChoice.TRAPEZOID_WPI,
                        new IncrementalProfiledController(
                                new TrapezoidProfileWPI(vel, accel),
                                feedback, mod, pTol, vTol)));
        m_chooser = new PolledEnumChooser<>(
                async,
                ProfileChoice.class,
                name,
                ProfileChoice.TRAPEZOID_100,
                this::setDelegate);
    }

    public ProfileChoice getChoice() {
        return m_chooser.get();
    }

    public void setDelegate(ProfileChoice choice) {
        m_selected = m_controllers.get(choice);
        // Must initialize when switching controllers.
        // For safety, use zero velocity.
        double measurement = m_measurement.getAsDouble();
        m_selected.init(new Model100(measurement, 0));
    }

    @Override
    public void init(Model100 measurement) {
        m_selected.init(measurement);
    }

    @Override
    public Result calculate(Model100 measurement, Model100 goal) {
        return m_selected.calculate(measurement, goal);
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
        m_chooser.close();
        for (ProfiledController c : m_controllers.values()) {
            c.close();
        }
    }
}
