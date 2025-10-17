package org.team100.lib.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Drive to the supplied target using a profile, so that the target is at
 * the specified relative bearing, e.g. the for intaking.
 * 
 * If the supplier starts delivering empties (e.g. the camera loses sight of the
 * goal), retain the old goal (forever).
 */
public class DriveToTranslationWithRelativeBearing extends MoveAndHold {
    /** verrrrrry loose. */
    private static final double THETA_TOLERANCE = 0.1;
    private final FieldLogger.Log m_field_log;
    private final Supplier<Optional<Translation2d>> m_targets;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;
    private final Rotation2d m_relativeBearing;

    private Pose2d m_goal;
    private ProfileReferenceR3 m_reference;
    private ReferenceController m_referenceController;

    public DriveToTranslationWithRelativeBearing(
            FieldLogger.Log fieldLogger,
            Supplier<Optional<Translation2d>> targetes,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile,
            Rotation2d relativeBearing) {
        m_field_log = fieldLogger;
        m_targets = targetes;
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_relativeBearing = relativeBearing;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        updateGoal();
        if (m_goal == null)
            return;
        m_reference = new ProfileReferenceR3(m_profile, "DriveToTranslationWithRelativeBearing");
        m_reference.setGoal(new ModelR3(m_goal));
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        if (m_goal == null || m_referenceController == null)
            return;
        m_reference.setGoal(new ModelR3(m_goal));
        m_referenceController.execute();
        m_field_log.m_log_ball.log(() -> new double[] {
                m_goal.getX(),
                m_goal.getY(),
                m_goal.getRotation().getRadians() });
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    /** For the runway path we can switch to "go to goal" after we're aligned. */
    public boolean thetaAligned() {
        if (m_goal == null)
            return false;
        Pose2d pose = m_drive.getPose();
        Rotation2d goalR = m_goal.getRotation();
        Rotation2d poseR = pose.getRotation();
        return Math.abs(goalR.minus(poseR).getRadians()) < THETA_TOLERANCE;
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
        m_targets.get().ifPresent(
                (target) -> m_goal = new Pose2d(
                        target,
                        heading(target)));
    }

    /** Robot heading to achieve the desired relative bearing to the target. */
    private Rotation2d heading(Translation2d target) {
        return absoluteBearing(m_drive.getPose(), target).minus(m_relativeBearing);
    }

    /** Field relative bearing from the robot to the target */
    private static Rotation2d absoluteBearing(Pose2d robot, Translation2d target) {
        return target.minus(robot.getTranslation()).getAngle();
    }
}
