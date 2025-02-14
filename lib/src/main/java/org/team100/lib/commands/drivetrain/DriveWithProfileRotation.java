package org.team100.lib.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Creates a profile to the translation of a note and follows it.
 * 
 * If the goal supplier runs empty, this remembers the previous goal for 1 sec,
 * and then gives up.
 */
public class DriveWithProfileRotation extends Command implements Glassy {

    public static double kThetaTolRad = Math.PI / 32;
    public static double kXTolM = 0.01;
    public static double kOmegaTolRad_s = Math.PI / 64;
    public static double kVTolM_S = 0.01;

    private final Supplier<Optional<Pose2d>> m_fieldRelativeGoalSupplier;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final SwerveKinodynamics m_limits;
    private Profile100 xProfile;
    private Profile100 yProfile;
    private final Profile100 thetaProfile;
    private final double dt = 0.02;
    private double sx = 1;
    private double sy = 1;

    private final FieldLogger.Log m_field_log;

    private Control100 xSetpoint;
    private Control100 ySetpoint;
    private Control100 thetaSetpoint;

    private Model100 m_xGoalRaw;
    private Model100 m_yGoalRaw;
    private Model100 m_thetaGoalRaw;

    SwerveModel currentGoalState;

    private final Supplier<Optional<Translation2d>> m_fieldRelativeGoal2;
    private Pose2d m_fieldRelativeGoal = null;


    private static Optional<Pose2d> m_goal;
    private int m_count;
    private Translation2d m_previousGoal;

    public DriveWithProfileRotation(
            FieldLogger.Log fieldLogger,
            Supplier<Optional<Translation2d>> fieldRelativeGoal,
            SwerveDriveSubsystem drivetrain,
            SwerveController controller,
            SwerveKinodynamics limits) {

        m_field_log = fieldLogger;
        m_fieldRelativeGoalSupplier = () -> m_goal;
        m_drive = drivetrain;
        m_controller = controller;
        m_limits = limits;
        xProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2() / 2,
                kXTolM);
        yProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2() / 2,
                kXTolM);
        thetaProfile = new TrapezoidProfile100(
                m_limits.getMaxAngleSpeedRad_S(),
                m_limits.getMaxAngleAccelRad_S2() / 50,
                kThetaTolRad);
        addRequirements(m_drive);

        kThetaTolRad = 2 * Math.PI;
        kOmegaTolRad_s = 6 * Math.PI;
        kXTolM = 0.05;
        m_fieldRelativeGoal2 = fieldRelativeGoal;

    }

    /**
     * Returns the current goal, or the previous one if the current one is newly
     * empty.
     */
    private Optional<Translation2d> getGoal() {
        return m_fieldRelativeGoal2.get();
        // Optional<Translation2d> optGoal = m_fieldRelativeGoal.get();
        // if (optGoal.isPresent()) {
        // // Supplier is ok, use this goal and reset the history mechanism.
        // m_previousGoal = optGoal.get();
        // m_count = 0;
        // return optGoal;
        // }
        // if (m_count > 50) {
        // // Supplier is empty and timer has expired.
        // return Optional.empty();
        // }
        // if (m_previousGoal == null) {
        // // Nothing to fall back to.
        // return Optional.empty();
        // }
        // m_count++;
        // return Optional.of(m_previousGoal);
    }

    /**
     * Returns the current goal, or the previous one if the current one is newly
     * empty.
     */
    private Optional<Pose2d> getGoalPose() {
        Optional<Translation2d> optGoal = getGoal();
        if (m_fieldRelativeGoal2.get().isPresent()) {
            return Optional
                    .of(new Pose2d(optGoal.get(), new Rotation2d(getThetaGoalRad(optGoal.get(), m_drive.getPose()))));
        }
        return Optional.empty();
    }

    @Override
    public void initialize() {
        m_goal = getGoalPose();

        xSetpoint = m_drive.getState().x().control();
        ySetpoint = m_drive.getState().y().control();
        thetaSetpoint = m_drive.getState().theta().control();

        Optional<Pose2d> opt = m_fieldRelativeGoalSupplier.get();
        if (opt.isEmpty()) {
            if (m_fieldRelativeGoal == null) {
                return;
            }
        } else {
            m_fieldRelativeGoal = opt.get();
        }

        m_xGoalRaw = new Model100(m_fieldRelativeGoal.getX(), 0);
        m_yGoalRaw = new Model100(m_fieldRelativeGoal.getY(), 0);

        double tx = xProfile.calculateWithETA(dt, xSetpoint.model(), m_xGoalRaw).etaS();
        double ty = yProfile.calculateWithETA(dt, ySetpoint.model(), m_yGoalRaw).etaS();
        double slowETA = Math.max(tx, ty);

        double maxV = m_limits.getMaxDriveVelocityM_S();
        double maxA = m_limits.getMaxDriveAccelerationM_S2() / 2;
        sx = TrapezoidProfile100.solveForSlowerETA(
                maxV, maxA, kXTolM,
                dt,
                xSetpoint.model(), m_xGoalRaw, slowETA, kVTolM_S);
        sy = TrapezoidProfile100.solveForSlowerETA(
                maxV, maxA, kXTolM,
                dt,
                ySetpoint.model(), m_yGoalRaw, slowETA, kVTolM_S);

        xProfile = xProfile.scale(sx);
        yProfile = yProfile.scale(sy);

        currentGoalState = null;

    }

    @Override
    public void execute() {
        m_goal = getGoalPose();

        Rotation2d currentRotation = m_drive.getPose().getRotation();
        // take the short path
        double measurement = currentRotation.getRadians();
        Optional<Pose2d> opt = m_fieldRelativeGoalSupplier.get();
        if (opt.isEmpty()) {
            if (m_fieldRelativeGoal == null) {
                return;
            }
        } else {
            m_fieldRelativeGoal = opt.get();
        }
        Rotation2d rotation = new Rotation2d(
                Math100.getMinDistance(measurement, m_fieldRelativeGoal.getRotation().getRadians()));
        // make sure the setpoint uses the modulus close to the measurement.
        thetaSetpoint = new Control100(
                Math100.getMinDistance(measurement, thetaSetpoint.x()),
                thetaSetpoint.v());

        m_thetaGoalRaw = new Model100(rotation.getRadians(), 0);
        m_xGoalRaw = new Model100(m_fieldRelativeGoal.getX(), 0);
        xSetpoint = xProfile.calculate(TimedRobot100.LOOP_PERIOD_S, xSetpoint.model(), m_xGoalRaw);

        m_yGoalRaw = new Model100(m_fieldRelativeGoal.getY(), 0);
        ySetpoint = yProfile.calculate(TimedRobot100.LOOP_PERIOD_S, ySetpoint.model(), m_yGoalRaw);
        m_field_log.m_log_target.log(() -> new double[] {
                m_fieldRelativeGoal.getX(),
                m_fieldRelativeGoal.getY(),
                m_fieldRelativeGoal.getRotation().getRadians() });

        thetaSetpoint = thetaProfile.calculate(TimedRobot100.LOOP_PERIOD_S, thetaSetpoint.model(), m_thetaGoalRaw);
        SwerveModel nextGoalState = new SwerveModel(xSetpoint.model(), ySetpoint.model(), thetaSetpoint.model());
        if (currentGoalState == null) {
            currentGoalState = nextGoalState;
        }
        FieldRelativeVelocity goal = m_controller.calculate(
                m_drive.getState(), currentGoalState, nextGoalState);
        currentGoalState = nextGoalState;
        m_drive.driveInFieldCoords(goal);

    }

    @Override
    public boolean isFinished() {
        if (!m_fieldRelativeGoalSupplier.get().isPresent() && m_fieldRelativeGoal == null) {
            return true;
        }
        double xError = m_xGoalRaw.x() - m_drive.getState().x().x();
        double yError = m_yGoalRaw.x() - m_drive.getState().y().x();
        double thetaError = m_thetaGoalRaw.x() - m_drive.getState().theta().x();
        return Math.abs(xError) < kXTolM
                && Math.abs(yError) < kXTolM
                && Math.abs(thetaError) < kThetaTolRad
                && atRest();
    }

    public boolean atRest() {
        return Math.abs(m_drive.getState().x().v()) < kVTolM_S
                && Math.abs(m_drive.getState().y().v()) < kVTolM_S
                && Math.abs(m_drive.getState().theta().v()) < kOmegaTolRad_s;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        xProfile = xProfile.scale(1 / sx);
        yProfile = yProfile.scale(1 / sy);
    }

    private double getThetaGoalRad(Translation2d goal, Pose2d pose) {
        if (Experiments.instance.enabled(Experiment.DriveToNoteWithRotation)) {
            // face the rear of the robot towards the goal.
            Translation2d toGoal = goal.minus(pose.getTranslation());
            switch (Identity.instance) {
                case COMP_BOT:
                case BLANK:
                    return toGoal.getAngle().getRadians() + Math.PI;
                default:
                    return toGoal.getAngle().getRadians();
            }
        } else {
            // leave the rotation alone
            return pose.getRotation().getRadians();
        }
    }
}
