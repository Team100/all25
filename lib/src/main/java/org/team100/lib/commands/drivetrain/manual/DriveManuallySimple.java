package org.team100.lib.commands.drivetrain.manual;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;

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
public class DriveManuallySimple extends Command implements Glassy {
    private static final boolean DEBUG = false;

    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    private final SwerveDriveSubsystem m_drive;
    private final FieldRelativeDriver m_defaultDriver;
    private final FieldRelativeDriver m_alternativeDriver;

    String currentManualMode = null;
    Supplier<Boolean> m_useAlternative;

    boolean wasUsingDefault = true;


    public DriveManuallySimple(
            Supplier<DriverControl.Velocity> twistSupplier,
            SwerveDriveSubsystem drive,
            FieldRelativeDriver driver,
            FieldRelativeDriver alterativeDriver,
            Supplier<Boolean> useAlteranative) {
        m_twistSupplier = twistSupplier;
        m_drive = drive;
        m_defaultDriver = driver;
        m_alternativeDriver = alterativeDriver;
        m_useAlternative = useAlteranative;
        
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.resetLimiter();
        SwerveModel p = m_drive.getState();
        m_defaultDriver.reset(p);
    }

    @Override
    public void execute() {
        // input in [-1,1] control units

        DriverControl.Velocity input = m_twistSupplier.get();
        SwerveModel state = m_drive.getState();

        if(m_useAlternative.get() == wasUsingDefault) { //we need to reset the driver because we switched modes from you WERE using default to now using alternative
            m_alternativeDriver.reset(m_drive.getState());
        }

        if(!wasUsingDefault == !m_useAlternative.get()) { //we weret using default and now we are
            m_defaultDriver.reset(m_drive.getState());
        }

        if(m_useAlternative.get()) {
            m_drive.driveInFieldCoords(m_alternativeDriver.apply(state, input));
        } else {
            m_drive.driveInFieldCoords(m_defaultDriver.apply(state, input));
        }

        wasUsingDefault = !m_useAlternative.get();

    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
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


    @Override
  public boolean isFinished() {
    // return m_wrist.atSetpoint();
    // return m_completeCommand.get();
    return false;
  }

    
    
}
