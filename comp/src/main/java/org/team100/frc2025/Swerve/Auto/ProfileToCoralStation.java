package org.team100.frc2025.Swerve.Auto;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.logging.LoggerFactory.StringLogger;
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


 //3.6 2.83
public class ProfileToCoralStation extends Command implements Glassy {
    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double kHeedRadiusM = 3;

    private final SwerveDriveSubsystem m_drive;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;

    private final StringLogger m_log_sector;
    private final Pose2dLogger m_log_goal;

    private Pose2d m_goal;
    private ProfileReference m_reference;
    private ReferenceController m_referenceController;
    Supplier<DriverControl.Velocity> m_velocityInput;

    boolean finished = false;

    public ProfileToCoralStation(
            LoggerFactory logger,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            SwerveController controller,
            HolonomicProfile profile,
            Supplier<DriverControl.Velocity> velocityInput) {
        LoggerFactory child = logger.child(this);
        m_log_sector = child.stringLogger(Level.TRACE, "sector");
        m_log_goal = child.pose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_heedRadiusM = heedRadiusM;
        m_controller = controller;
        m_profile = profile;
        m_velocityInput = velocityInput;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(kHeedRadiusM);
        Pose2d currentPose = m_drive.getPose();
        FieldSector currentSector = FieldConstants.getSector(currentPose);
        m_log_sector.log(() -> currentSector.name());

        Pose2d goalTranslationLeft = new Pose2d(1.2, 7.00, Rotation2d.fromDegrees(-54));
        Pose2d goalTranslationRight = new Pose2d(1.2, 1.05, Rotation2d.fromDegrees(54));

        ArrayList<Pose2d> poses = new ArrayList<>();

        poses.add(goalTranslationLeft);
        poses.add(goalTranslationRight);

        m_goal = currentPose.nearest(poses);

        finished = false;
        m_log_goal.log(() -> m_goal);
        m_reference = new ProfileReference(m_profile);
        m_reference.setGoal(new SwerveModel(m_goal));
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        if (m_goal == null || m_referenceController == null)
            return;

        if(m_velocityInput.get().x() >= 0.1 || m_velocityInput.get().y() >= 0.1 ){
            finished = true;
        }

        m_reference.setGoal(new SwerveModel(m_goal));
        m_referenceController.execute();
    }

    @Override
    public boolean isFinished() {

        return (m_referenceController != null && m_referenceController.isFinished()) || finished ;
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("*************I FINISHED EMBBARKING********************");
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
        m_goal = null;
    }

}
