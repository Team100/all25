package org.team100.lib.commands.drivetrain.manual;

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
public class DriveAdjustCoral extends Command implements Glassy {
    private static final boolean DEBUG = false;

    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    private final SwerveDriveSubsystem m_drive;
    private final Driver m_defaultDriver;
    String currentManualMode = null;
    Supplier<Boolean> m_completeCommand;

    FieldRelativeDriver m_driver;

    public DriveAdjustCoral(
            Supplier<DriverControl.Velocity> twistSupplier,
            SwerveDriveSubsystem drive,
            Supplier<Boolean> completeCommand,
            FieldRelativeDriver driver) {
        m_twistSupplier = twistSupplier;
        m_drive = drive;
        m_defaultDriver = stop();
        m_driver = driver;
        m_completeCommand = completeCommand;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.resetLimiter();
        SwerveModel p = m_drive.getState();
        m_driver.reset(p);
    }

    @Override
    public void execute() {
        // input in [-1,1] control units
        DriverControl.Velocity input = m_twistSupplier.get();
        SwerveModel state = m_drive.getState();
        // System.out.println("IM RUNN");
        m_drive.driveInFieldCoords(m_driver.apply(state, input));

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
    return m_completeCommand.get();
  }

    
    
}
