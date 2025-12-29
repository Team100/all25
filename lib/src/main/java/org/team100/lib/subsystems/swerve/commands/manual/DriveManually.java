package org.team100.lib.subsystems.swerve.commands.manual;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual drivetrain control.
 * 
 * Provides four manual control modes:
 * 
 * -- raw module state
 * -- robot-relative
 * -- field-relative
 * -- field-relative with rotation control
 * 
 * Use the mode supplier to choose which mode to use, e.g. using a Sendable
 * Chooser.
 */
public class DriveManually extends Command {
    /**
     * While driving manually, pay attention to tags even if they are somewhat far
     * away.
     */
    private static final double HEED_RADIUS_M = 6.0;

    private static final SendableChooser<String> m_manualModeChooser = new NamedChooser<>("Manual Drive Mode");

    private Supplier<String> m_mode;
    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<Velocity> m_twistSupplier;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveLimiter m_limiter;
    private final Map<String, DriverAdapter> m_drivers;
    private final DriverAdapter m_defaultDriver;
    String currentManualMode = null;

    public DriveManually(
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter) {
        m_mode = m_manualModeChooser::getSelected;
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_defaultDriver = new StopDriver(m_drive);
        m_drivers = new ConcurrentHashMap<>();
        SmartDashboard.putData(m_manualModeChooser);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        // make sure the limiter knows what we're doing
        m_limiter.updateSetpoint(m_drive.getVelocity());
        ModelSE2 p = m_drive.getState();
        for (DriverAdapter d : m_drivers.values()) {
            d.reset(p);
        }
    }

    @Override
    public void execute() {
        String manualMode = m_mode.get();
        if (manualMode == null) {
            return;
        }

        if (!(manualMode.equals(currentManualMode))) {
            currentManualMode = manualMode;
            // there's state in there we'd like to forget
            ModelSE2 p = m_drive.getState();
            for (DriverAdapter d : m_drivers.values()) {
                d.reset(p);
            }
        }

        // input in [-1,1] control units
        Velocity input = m_twistSupplier.get();
        ModelSE2 state = m_drive.getState();
        DriverAdapter d = m_drivers.getOrDefault(manualMode, m_defaultDriver);
        d.apply(state, input);

    }

    /**
     * Override the driver mode.
     * 
     * For testing only.
     */
    void overrideMode(Supplier<String> mode) {
        m_mode = mode;
    }

    /** Register a driver for module state mode */
    public void register(String name, boolean isDefault, ModuleStateDriver d) {
        addName(name, isDefault);
        m_drivers.put(name, new ModuleStateAdapter(m_drive, d));
    }

    /** Register a driver for robot-relative speed mode */
    public void register(String name, boolean isDefault, ChassisSpeedDriver d) {
        addName(name, isDefault);
        m_drivers.put(name, new ChassisSpeedAdapter(m_drive, d));
    }

    /** Register a driver for field-relative speed mode */
    public void register(String name, boolean isDefault, FieldRelativeDriver d) {
        addName(name, isDefault);
        m_drivers.put(name, new FieldRelativeAdapter(m_limiter, m_drive, d));
    }

    //////////////

    private void addName(String name, boolean isDefault) {
        m_manualModeChooser.addOption(name, name);
        if (isDefault)
            m_manualModeChooser.setDefaultOption(name, name);
    }
}
