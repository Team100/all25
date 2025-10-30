package org.team100.lib.commands.r3;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.controller.r3.VelocityReferenceControllerR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.r3.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.VelocitySubsystemR3;
import org.team100.lib.targeting.TargetUtil;

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
    private final DoubleArrayLogger m_log_field_ball;
    private final Supplier<Optional<Translation2d>> m_targets;
    private final VelocitySubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final HolonomicProfile m_profile;
    private final Rotation2d m_relativeBearing;
    private final Translation2d m_relativeTranslation;

    private Pose2d m_goal;
    private Translation2d m_translation;
    private ProfileReferenceR3 m_reference;
    private VelocityReferenceControllerR3 m_referenceController;

    public DriveToTranslationWithRelativeBearing(
            LoggerFactory field,
            Supplier<Optional<Translation2d>> targetes,
            VelocitySubsystemR3 drive,
            ControllerR3 controller,
            HolonomicProfile profile,
            Rotation2d relativeBearing,
            Translation2d relativeTranslation) {
        m_log_field_ball = field.doubleArrayLogger(Level.COMP, "ball");
        m_targets = targetes;
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_relativeBearing = relativeBearing;
        m_relativeTranslation = relativeTranslation;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        updateGoal();
        if (m_goal == null)
            return;
        m_reference = new ProfileReferenceR3(m_profile, "DriveToTranslationWithRelativeBearing");
        m_reference.setGoal(new ModelR3(m_goal));
        m_referenceController = new VelocityReferenceControllerR3(
                m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        if (m_goal == null || m_referenceController == null)
            return;
        m_reference.setGoal(new ModelR3(m_goal));
        m_referenceController.execute();
        m_log_field_ball.log(() -> new double[] {
                m_goal.getX(),
                m_goal.getY(),
                m_goal.getRotation().getRadians() });
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    /** For the runway path we can switch to "go to goal" after we're aligned. */
    public boolean thetaAligned() {
        if (m_goal == null)
            return false;
        Pose2d pose = m_drive.getState().pose();
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
                (target) -> {
                    m_translation = target;
                    Rotation2d heading = heading(target);
                    m_goal = new Pose2d(
                            target.minus(m_relativeTranslation.rotateBy(m_drive.getState().pose().getRotation())),
                            heading);
                });
    }

    /** Robot heading to achieve the desired relative bearing to the target. */
    private Rotation2d heading(Translation2d target) {
        return TargetUtil.absoluteBearing(
                m_drive.getState().pose().getTranslation(),
                target).minus(m_relativeBearing);
    }
}
