package org.team100.frc2025.Swerve.Auto;

import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to the supplied pose using a profile.
 * 
 * If the supplier starts delivering empties, retain the old goal.
 */
public class ProfileToDestination extends Command implements Glassy {
    
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;

    private Pose2d m_goal;
    private ProfileReference m_reference;
    private ReferenceController m_referenceController;
    private FieldSector m_destinationSector;
    private ReefDestination m_reefDestination;

    public ProfileToDestination(
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile,
            FieldSector destinationSector,
            ReefDestination reefDestination) {
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_destinationSector = destinationSector;
        m_reefDestination = reefDestination;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_drive.getPose();
        // FieldSector currentSector = m_destinationSector;
        Translation2d destination = FieldConstants.getOrbitDestination(m_destinationSector, m_reefDestination, 1.3);
        Rotation2d heading = FieldConstants.getSectorAngle(m_destinationSector).rotateBy(Rotation2d.fromDegrees(180));
        m_goal = new Pose2d(destination, heading);


        // if (m_goal == null)
        //     return;
        m_reference = new ProfileReference(m_profile);
        m_reference.setGoal(new SwerveModel(m_goal));
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        // updateGoal();
        if (m_goal == null || m_referenceController == null)
            return;
        m_reference.setGoal(new SwerveModel(m_goal));
        m_referenceController.execute();
        // m_field_log.m_log_target.log(() -> new double[] {
        //         m_goal.getX(),
        //         m_goal.getY(),
        //         m_goal.getRotation().getRadians() });
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

  

}
