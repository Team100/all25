package org.team100.lib.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to the supplied pose using a profile.
 * 
 * If the supplier starts delivering empties, retain the old goal.
 */
public class DriveToPoseWithProfile extends Command implements Glassy {
    private final FieldLogger.Log m_field_log;
    private final Supplier<Optional<Pose2d>> m_goals;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;

    private Pose2d m_goal;
    private ProfileReference m_reference;
    private ReferenceController m_referenceController;

    public DriveToPoseWithProfile(
            FieldLogger.Log fieldLogger,
            Supplier<Optional<Pose2d>> goals,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile) {
        m_field_log = fieldLogger;
        m_goals = goals;
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        updateGoal();
        if (m_goal == null)
            return;
        m_reference = new ProfileReference(m_profile);
        m_reference.setGoal(new SwerveModel(m_goal));
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        updateGoal();
        if (m_goal == null || m_referenceController == null)
            return;
        m_field_log.m_log_target.log(() -> new double[] {
                m_goal.getX(),
                m_goal.getY(),
                m_goal.getRotation().getRadians() });
        m_reference.setGoal(new SwerveModel(m_goal));
        m_referenceController.execute();
    }

    @Override
    public boolean isFinished() {
        return m_referenceController != null && m_referenceController.isFinished();
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
        m_goals.get().ifPresent((x) -> m_goal = x);
    }

}
