package org.team100.lib.commands.drivetrain.manual;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.util.NamedChooser;
import org.team100.lib.util.Util;

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
public class DriveManually extends Command implements Glassy {
    /** While driving manually, pay attention to tags even if they are far away. */
    private static final double kHeedRadiusM = 12.0;

    private static final boolean DEBUG = false;

    private static final SendableChooser<String> m_manualModeChooser = new NamedChooser<>("Manual Drive Mode") {
    };

    private Supplier<String> m_mode;
    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveDriveSubsystem m_drive;
    private final Map<String, Driver> m_drivers;
    private final Driver m_defaultDriver;
    String currentManualMode = null;

    public DriveManually(
            Supplier<DriverControl.Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive) {
        m_mode = m_manualModeChooser::getSelected;
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_defaultDriver = stop();
        m_drivers = new ConcurrentHashMap<>();
        SmartDashboard.putData(m_manualModeChooser);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(kHeedRadiusM);
        m_drive.resetLimiter();
        SwerveModel p = m_drive.getState();
        for (Driver d : m_drivers.values()) {
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
            SwerveModel p = m_drive.getState();
            for (Driver d : m_drivers.values()) {
                d.reset(p);
            }
        }

        // input in [-1,1] control units
        DriverControl.Velocity input = m_twistSupplier.get();
        SwerveModel state = m_drive.getState();
        Driver d = m_drivers.getOrDefault(manualMode, m_defaultDriver);
        d.apply(state, input);

    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
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
        m_drivers.put(
                name,
                new Driver() {
                    public void apply(SwerveModel s, DriverControl.Velocity t) {
                        if (DEBUG)
                            Util.printf("ModuleStateDriver %s\n", t);
                        m_drive.setRawModuleStates(d.apply(t));
                    }

                    public void reset(SwerveModel p) {
                        //
                    }
                });
    }

    /** Register a driver for robot-relative speed mode */
    public void register(String name, boolean isDefault, ChassisSpeedDriver d) {
        addName(name, isDefault);
        m_drivers.put(
                name,
                new Driver() {
                    public void apply(SwerveModel s, DriverControl.Velocity t) {
                        if (DEBUG)
                            Util.printf("ChassisSpeedDriver %s\n", t);
                        m_drive.setChassisSpeeds(d.apply(s, t));
                    }

                    public void reset(SwerveModel p) {
                        d.reset(p);
                    }
                });
    }

    /** Register a driver for field-relative speed mode */
    public void register(String name, boolean isDefault, FieldRelativeDriver d) {
        addName(name, isDefault);
        m_drivers.put(
                name,
                new Driver() {
                    public void apply(SwerveModel s, DriverControl.Velocity t) {
                        if (DEBUG)
                            Util.printf("FieldRelativeDriver %s\n", t);
                        m_drive.driveInFieldCoords(d.apply(s, t));
                    }

                    public void reset(SwerveModel p) {
                        d.reset(p);
                    }
                });
    }

    //////////////

    private Driver stop() {
        return new Driver() {
            public void apply(SwerveModel s, DriverControl.Velocity t) {
                m_drive.stop();
            }

            public void reset(SwerveModel p) {
                //
            }
        };
    }

    private void addName(String name, boolean isDefault) {
        m_manualModeChooser.addOption(name, name);
        if (isDefault)
            m_manualModeChooser.setDefaultOption(name, name);
    }
}
