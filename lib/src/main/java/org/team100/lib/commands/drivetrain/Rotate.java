package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Rotate in place to the specified angle.
 * 
 * This is probably only useful for testing.
 */
public class Rotate extends MoveAndHold {
    /** For testing */
    private static final boolean DEBUG = false;
    private static final double THETA_TOLERANCE_RAD = 0.02;
    // don't try to rotate at max speed
    private static final double SPEED = 0.5;

    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Rotation2d m_target;
    private final HolonomicProfile m_profile;

    private ProfileReferenceR3 m_reference;
    private ReferenceController m_referenceController;

    public Rotate(
            SwerveDriveSubsystem drive,
            SwerveController controller,
            SwerveKinodynamics swerveKinodynamics,
            double targetAngleRadians) {
        m_drive = drive;
        m_controller = controller;
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = new Rotation2d(targetAngleRadians);
        m_profile = HolonomicProfile.trapezoidal(
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxDriveAccelerationM_S2(),
                0.01,
                m_swerveKinodynamics.getMaxAngleSpeedRad_S() * SPEED,
                m_swerveKinodynamics.getMaxAngleAccelRad_S2() * SPEED,
                THETA_TOLERANCE_RAD);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (DEBUG)
            System.out.println("Rotate initialize");
        Pose2d measurement = m_drive.getPose();
        // if we use the initial measurement x and y as the target, and we're moving,
        // then we make a u-turn to get back to the arbitrary place when we pushed the
        // button.
        // instead, pick a goal at the stopping distance in the current direction.
        Translation2d dx = m_drive.getVelocity().stopping(m_swerveKinodynamics.getMaxDriveAccelerationM_S2());
        Pose2d goal = new Pose2d(measurement.getX() + dx.getX(), measurement.getY() + dx.getY(), m_target);
        m_reference = new ProfileReferenceR3(m_profile, "rotate");
        m_reference.setGoal(new ModelR3(goal));
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        if (DEBUG)
            System.out.println("Rotate execute");
        if (m_referenceController != null)
            m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    @Override
    public void end(boolean isInterupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
    }
}
