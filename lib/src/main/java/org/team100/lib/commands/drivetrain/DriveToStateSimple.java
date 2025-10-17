package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.ConstantReferenceR3;
import org.team100.lib.reference.ReferenceR3;
import org.team100.lib.state.ModelR3;

/**
 * Given a swerve state, drive there using only the feedback in the holonomic
 * controller.
 * 
 * There's no profile, no trajectory, just give it a point
 * you want to go to, and it will go there. It makes no attempt to
 * impose feasibility constraints or coordinate the axes, so it tries
 * to use the setpoint generator to moderate the output.
 */
public class DriveToStateSimple extends MoveAndHold {
    private final SwerveController m_controller;
    private final SwerveDriveSubsystem m_drive;
    private final ReferenceR3 m_reference;

    private ReferenceController m_referenceController;

    public DriveToStateSimple(
            SwerveController controller,
            SwerveDriveSubsystem drive,
            ModelR3 goal) {
        m_controller = controller;
        m_drive = drive;
        m_reference = new ConstantReferenceR3(goal);
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
    public boolean isDone() {
        return m_referenceController.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
