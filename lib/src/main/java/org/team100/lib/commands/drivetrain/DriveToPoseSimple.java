package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.reference.ConstantReference;
import org.team100.lib.reference.SwerveReference;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Given a pose, drive there using only the holonomic controller,
 * no profile, no trajectory.  This doesn't seem to work well to follow
 * a trajectory or profile, don't use it for that, just give it a point
 * you want to go to, and it will go there.  It makes no attempt to
 * impose feasibility constraints or coordinate the axes, so it tries
 * to use the setpoint generator to moderate the output.
 */
public class DriveToPoseSimple extends Command implements Glassy {
    private final SwerveController m_controller;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveReference m_reference;

    private ReferenceController m_referenceController;

    public DriveToPoseSimple(
            SwerveController controller,
            SwerveDriveSubsystem drive,
            SwerveModel goal) {
        m_controller = controller;
        m_drive = drive;
        m_reference = new ConstantReference(goal);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public boolean isFinished() {
        return m_referenceController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
