package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReference;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to the supplied pose using a profile.
 * 
 * If the supplier starts delivering empties, retain the old goal.
 */
public class DriveToPoseWithProfile extends Command implements Glassy {
    private final FieldLogger.Log m_field_log;
    private final Supplier<SwerveModel> m_goals;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;
    private final boolean m_careAboutPosition;

    private SwerveModel m_goal;
    private ProfileReference m_reference;
    private ReferenceController m_referenceController;

    public DriveToPoseWithProfile(
            FieldLogger.Log fieldLogger,
            Supplier<SwerveModel> goal,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile, 
            boolean careAboutPosition) {
        m_field_log = fieldLogger;
        m_goals = goal;
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_careAboutPosition = careAboutPosition;
        addRequirements(m_drive);
    }

    public DriveToPoseWithProfile(
            FieldLogger.Log fieldLogger,
            Supplier<SwerveModel> goal,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile) {
        this(fieldLogger, goal,drive,controller, profile, true);
    }

    @Override
    public void initialize() {
        updateGoal();
        if (m_goal == null)
            return;
        m_reference = new ProfileReference(m_profile);
        m_reference.setGoal(m_goal);
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        updateGoal();
        if (m_goal == null || m_referenceController == null)
            return;
        m_reference.setGoal(m_goal);
        m_referenceController.execute();
        m_field_log.m_log_target.log(() -> new double[] {
                m_goal.x().x(),
                m_goal.y().x(),
                m_goal.theta().x() });
    }

    @Override
    public boolean isFinished() {
        return m_referenceController != null && (m_referenceController.isFinished() || (!m_careAboutPosition && m_referenceController.isDone()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
        m_goal = null;
    }

    private void updateGoal() {
        m_goal = m_goals.get();
    }

}
