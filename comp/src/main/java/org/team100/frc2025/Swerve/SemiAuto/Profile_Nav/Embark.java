package org.team100.frc2025.Swerve.SemiAuto.Profile_Nav;

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
public class Embark extends Command implements Glassy {
    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double kHeedRadiusM = 3;

    private final SwerveDriveSubsystem m_drive;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;
    private final FieldSector m_targetSector;
    private final ReefDestination m_destination;
    private final Supplier<ScoringPosition> m_scoringPositionSupplier;
    private final double m_radius;

    private final StringLogger m_log_sector;
    private final Pose2dLogger m_log_goal;

    private Pose2d m_goal;
    private ProfileReference m_reference;
    private ReferenceController m_referenceController;

    private ReefPoint m_point;

    public Embark(
            LoggerFactory logger,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            SwerveController controller,
            HolonomicProfile profile,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> scoringPositionSupplier,
            ReefPoint point) {
        this(
                logger,
                drive,
                heedRadiusM,
                controller,
                profile,
                targetSector,
                destination,
                scoringPositionSupplier,
                0,
                point);
    }

    public Embark(
            LoggerFactory logger,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            SwerveController controller,
            HolonomicProfile profile,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> scoringPositionSupplier,
            double radius,
            ReefPoint point) {
        LoggerFactory child = logger.child(this);
        m_log_sector = child.stringLogger(Level.TRACE, "sector");
        m_log_goal = child.pose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_heedRadiusM = heedRadiusM;
        m_controller = controller;
        m_profile = profile;
        m_scoringPositionSupplier = scoringPositionSupplier;
        m_targetSector = targetSector;
        m_destination = destination;
        m_radius = radius;
        m_point = point;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(kHeedRadiusM);
        Pose2d currentPose = m_drive.getPose();
        FieldSector currentSector = FieldConstants.getSector(currentPose);
        m_log_sector.log(() -> currentSector.name());

        ScoringPosition scoringPosition = m_scoringPositionSupplier.get();

        double radius = 0;

        if (m_radius != 0) {
            radius = m_radius;
        } else {
            if (m_destination == ReefDestination.CENTER) {
                radius = 1.2;
            }

            if (scoringPosition == ScoringPosition.L4) {
                radius = 1.485;

                switch (m_point) {
                    case A:
                        radius = 1.485;
                        break;
                    case B:
                        radius = 1.455;
                        break;
                    case C:
                        radius = 1.430;
                        break;
                    case D:
                        radius = 1.430;
                        break;
                    case E:
                        radius = 1.430;
                        break;
                    case F:
                        radius = 1.430;
                        break;
                    case G:
                        radius = 1.485;
                        break;
                    case H:
                        radius = 1.450;
                        break;
                    case I:
                        radius = 1.450;
                        break;
                    case J:
                        radius = 1.485;
                        break;
                    case K:
                        radius = 1.460;
                        break;
                    case L:
                        radius = 1.485;
                        break;
                    default:
                        break;
                }
            }

            if (scoringPosition == ScoringPosition.L3) {
                switch (m_point) {
                    case A:
                        radius = 1.34;
                        break;
                    case B:
                        radius = 1.34;
                        break;
                    case C:
                        radius = 1.34;
                        break;
                    case D:
                        radius = 1.34;
                        break;
                    case E:
                        radius = 1.34;
                        break;
                    case F:
                        radius = 1.34;
                        break;
                    case G:
                        radius = 1.34;
                        break;
                    case H:
                        radius = 1.32;
                        break;
                    case I:
                        radius = 1.34;
                        break;
                    case J:
                        radius = 1.34;
                        break;
                    case K:
                        radius = 1.34;
                        break;
                    case L:
                        radius = 1.34;
                        break;
                    default:
                        break;
                }
            }

            if (scoringPosition == ScoringPosition.L2) {
                radius = 1.33;
            }

            if(scoringPosition == ScoringPosition.NONE){
                radius = 3;
            }
        }

        Translation2d destination = FieldConstants.getOrbitDestination(m_targetSector, m_destination, radius);
        Rotation2d heading = FieldConstants.getSectorAngle(m_targetSector).rotateBy(Rotation2d.fromDegrees(180));

        m_goal = new Pose2d(destination, heading);
        m_log_goal.log(() -> m_goal);
        m_reference = new ProfileReference(m_profile);
        m_reference.setGoal(new SwerveModel(m_goal));
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    @Override
    public void execute() {
        if (m_goal == null || m_referenceController == null)
            return;
        m_reference.setGoal(new SwerveModel(m_goal));
        m_referenceController.execute();
    }

    @Override
    public boolean isFinished() {
        return m_referenceController != null && m_referenceController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("*************I FINISHED EMBBARKING********************");
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
        m_goal = null;
    }

}
