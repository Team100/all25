package org.team100.frc2025.Swerve.SemiAuto.Profile_Nav;

import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.field.FieldPoint2024;
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
public class Embark extends Command implements Glassy {
    
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;

    private Pose2d m_goal;
    private ProfileReference m_reference;
    private ReferenceController m_referenceController;

    private FieldSector m_targetSector;
    private ReefDestination m_destination;
    private Supplier<ScoringPosition> m_scoringPositionSupplier;
    private double m_radius;   

    private ScoringPosition m_scoringPosition = ScoringPosition.NONE;
    public Embark(
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> scoringPositionSupplier) {
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_scoringPositionSupplier = scoringPositionSupplier;
        m_targetSector = targetSector;
        m_destination = destination;
        m_radius = 0;
        addRequirements(m_drive);
    }

    public Embark(
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> scoringPositionSupplier,
            double radius) {
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_scoringPositionSupplier = scoringPositionSupplier;
        m_targetSector = targetSector;
        m_destination = destination;
        m_radius = radius;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_drive.getPose();
        FieldSector currentSector = FieldConstants.getSector(currentPose);
        m_scoringPosition = m_scoringPositionSupplier.get();
        System.out.println(m_scoringPosition);

        double radius = 0;

        if(m_radius != 0){
            radius = m_radius;
        } else {
            if(m_destination == ReefDestination.CENTER){
                radius = 1.2;
            }   
    
            if(m_scoringPosition == ScoringPosition.L4){
                radius = 1.42;
            }
    
            if(m_scoringPosition == ScoringPosition.L3){
                radius = 1.35;
            }
    
            if(m_scoringPosition == ScoringPosition.L2){
                radius = 1.35;
            }
        }

       
        

        Translation2d destination = FieldConstants.getOrbitDestination(m_targetSector, m_destination, radius);
        Rotation2d heading = FieldConstants.getSectorAngle(m_targetSector).rotateBy(Rotation2d.fromDegrees(180));


        m_goal = new Pose2d(destination, heading);

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
        System.out.println("*************I FINISHED EMBBARKING********************");
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
        m_goal = null;
    }

  

}
