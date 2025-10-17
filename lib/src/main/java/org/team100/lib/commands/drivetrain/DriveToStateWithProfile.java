package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;

/**
 * Drive to the supplied goal state using a profile. Allows the goal to change
 * after initialization.
 */
public class DriveToStateWithProfile extends MoveAndHold {
    private final FieldLogger.Log m_field_log;
    private final Supplier<ModelR3> m_goals;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;

    private ModelR3 m_goal;
    private ProfileReferenceR3 m_reference;
    private ReferenceController m_referenceController;

    public DriveToStateWithProfile(
            FieldLogger.Log fieldLogger,
            Supplier<ModelR3> goal,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile) {
        m_field_log = fieldLogger;
        m_goals = goal;
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_goal = m_goals.get();
        if (m_goal == null)
            return;
        m_reference = new ProfileReferenceR3(m_profile, "Drive to pose with profile");
        m_reference.setGoal(m_goal);
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        m_goal = m_goals.get();
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
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
        m_goal = null;
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

}
