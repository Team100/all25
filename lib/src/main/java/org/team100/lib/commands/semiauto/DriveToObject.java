package org.team100.lib.commands.semiauto;

import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.planner.DriveUtil;
import org.team100.lib.planner.ForceViz;
import org.team100.lib.util.Arg;
import org.team100.lib.util.Debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to the nearest object, turning so that the intake side arrives first.
 * 
 * If the robot is between two objects, it switches between them and makes no
 * progress. TODO: fix that.
 * 
 * This never finishes; run it with an Intake command as the deadline.
 */
public class DriveToObject extends Command {
    /** Alignment tolerance for the intake */
    public static final double kIntakeAdmittanceRad = 0.2;

    private final DriveSubsystemInterface m_drive;
    private final Supplier<Translation2d> m_camera;
    private final boolean m_debug;
    private final DriveUtil m_driveUtil;

    /**
     * 
     * @param swerveKinodynamics
     * @param drive
     * @param camera             given the robot pose, return the field-relative
     *                           position of the closest object, or null if none
     *                           nearby
     * @param tactics            given the desired velocity, return avoidance
     *                           velocity
     * @param viz
     * @param debug
     */
    public DriveToObject(
            SwerveKinodynamics swerveKinodynamics,
            DriveSubsystemInterface drive,
            Supplier<Translation2d> camera,
            UnaryOperator<FieldRelativeVelocity> tactics,
            ForceViz viz,
            boolean debug) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        m_drive = drive;
        m_camera = camera;
        m_debug = debug && Debug.enable();
        m_driveUtil = new DriveUtil(swerveKinodynamics, drive, viz, tactics, m_debug);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToObject");

        // where are we with respect to the goal?

        // TODO: use a future estimated pose to account for current velocity

        Pose2d pose = m_drive.getPose();

        goToGoal(pose);
    }

    /**
     * Go to the closest object, irrespective of the age of the sighting (since objects
     * don't move and sightings are all pretty new)
     */
    private void goToGoal(Pose2d pose) {

        // TODO: remember and prefer the previous fixation, unless some new sighting is
        // much better.

        Translation2d closestSighting = m_camera.get();
        if (closestSighting == null) {
            // no nearby object, no need to move
            m_drive.drive(m_driveUtil.finish(new FieldRelativeVelocity(0, 0, 0)));
            return;
        }

        // found an object

        FieldRelativeVelocity desired = m_driveUtil.goToGoalAligned(
                kIntakeAdmittanceRad,
                pose,
                closestSighting);

        m_drive.drive(desired);
    }
}
