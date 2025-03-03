package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathPlanner;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.ScheduleGenerator;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/**
 * joel 20240311: this class no longer applies default constraints (drive, yaw,
 * centripetal) so if you want those, supply them.
 */
public class TrajectoryPlanner {
    private static final double kMaxDx = 0.0127; // m
    private static final double kMaxDy = 0.0127; // m
    private static final double kMaxDTheta = Math.toRadians(1.0);
    // if we try to start a trajectory while respecting initial velocity, but the
    // initial velocity is less than 0.01 m/s, just treat it as rest-to-rest.
    private static final double VELOCITY_EPSILON = 1e-2;

    private final ScheduleGenerator m_scheduleGenerator;

    public TrajectoryPlanner(List<TimingConstraint> constraints) {
        m_scheduleGenerator = new ScheduleGenerator(constraints);
    }

    /** A square counterclockwise starting with +x. */
    public List<Trajectory100> square(Pose2d p0) {
        Pose2d p1 = p0.plus(new Transform2d(1, 0, Rotation2d.kZero));
        Pose2d p2 = p0.plus(new Transform2d(1, 1, Rotation2d.kZero));
        Pose2d p3 = p0.plus(new Transform2d(0, 1, Rotation2d.kZero));
        return List.of(
                restToRest(p0, p1),
                restToRest(p1, p2),
                restToRest(p2, p3),
                restToRest(p3, p0));
    }

    /** Make a square that gets a reset starting point at each corner. */
    public List<Function<Pose2d, Trajectory100>> permissiveSquare() {
        return List.of(
                x -> restToRest(x, x.plus(new Transform2d(1, 0, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(0, 1, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(-1, 0, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(0, -1, Rotation2d.kZero))));
    }

    /** From current to x+1 */
    public Trajectory100 line(Pose2d initial) {
        return restToRest(
                initial,
                initial.plus(new Transform2d(1, 0, Rotation2d.kZero)));
    }

    public Trajectory100 restToRest(
            List<Pose2d> waypoints,
            List<Rotation2d> headings) {
        return generateTrajectory(
                waypoints,
                headings,
                0.0,
                0.0);
    }

    public Trajectory100 restToRest(
            List<Pose2d> waypoints,
            List<Rotation2d> headings,
            List<Double> mN) {
        return generateTrajectory(
                waypoints,
                headings,
                0.0,
                0.0,
                mN);
    }

    public Trajectory100 movingToRest(SwerveModel startState, Pose2d end) {
        return movingToMoving(startState, new SwerveModel(end));
    }
    
    public Trajectory100 movingToMoving(SwerveModel startState, SwerveModel endState) {
        
        if (Math.abs(startState.velocity().norm()) < VELOCITY_EPSILON && Math.abs(endState.velocity().norm()) < VELOCITY_EPSILON) {
            return restToRest(startState.pose(), endState.pose());
        }
        Translation2d currentTranslation = startState.translation();
        FieldRelativeVelocity currentSpeed = startState.velocity();
        FieldRelativeVelocity endSpeed = endState.velocity();

        Translation2d goalTranslation = endState.translation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        Rotation2d startingAngle = currentSpeed.angle().orElse(angleToGoal);
        try {
            return generateTrajectory(
                    List.of(
                            new Pose2d(
                                    currentTranslation,
                                    startingAngle),
                            new Pose2d(
                                    goalTranslation,
                                    angleToGoal)),
                    List.of(
                            startState.rotation(),
                            endState.rotation()),
                    Math.abs(currentSpeed.norm()),
                    endSpeed.norm());
        } catch (TrajectoryGenerationException e) {
            Util.warn("Trajectory Generation Exception");
            return new Trajectory100();
        }
    }

    /**
     * Produces straight lines from start to end.
     */
    public Trajectory100 restToRest(
            Pose2d start,
            Pose2d end) {
        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        try {
            return restToRest(
                    List.of(
                            new Pose2d(currentTranslation, angleToGoal),
                            new Pose2d(goalTranslation, angleToGoal)),
                    List.of(start.getRotation(), end.getRotation()));
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }

    /**
     * If you want a max velocity or max accel constraint, use ConstantConstraint.
     */
    public Trajectory100 generateTrajectory(
            List<Pose2d> waypoints,
            List<Rotation2d> headings,
            double start_vel,
            double end_vel) {
        try {
            // Create a path from splines.
            Path100 path = PathPlanner.pathFromWaypointsAndHeadings(
                    waypoints, headings, kMaxDx, kMaxDy, kMaxDTheta);
            // Generate the timed trajectory.
            return m_scheduleGenerator.timeParameterizeTrajectory(
                    path,
                    kMaxDx,
                    start_vel,
                    end_vel);
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            Util.warn("Bad trajectory input!!");
            // print the stack trace if you want to know who is calling
            // e.printStackTrace();
            return new Trajectory100();
        }
    }

    public Trajectory100 generateTrajectory(
            Path100 path,
            double start_vel,
            double end_vel) {
        try {
            // Generate the timed trajectory.
            return m_scheduleGenerator.timeParameterizeTrajectory(
                    path,
                    kMaxDx,
                    start_vel,
                    end_vel);
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            Util.warn("Bad trajectory input!!");
            // print the stack trace if you want to know who is calling
            // e.printStackTrace();
            return new Trajectory100();
        }
    }

    public Trajectory100 generateTrajectory(
            List<Pose2d> waypoints,
            List<Rotation2d> headings,
            double start_vel,
            double end_vel,
            List<Double> mN) {
        try {
            // Create a path from splines.
            Path100 path = PathPlanner.pathFromWaypointsAndHeadings(
                    waypoints, headings, kMaxDx, kMaxDy, kMaxDTheta, mN);
            // Generate the timed trajectory.
            return m_scheduleGenerator.timeParameterizeTrajectory(
                    path,
                    kMaxDx,
                    start_vel,
                    end_vel);
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            Util.warn("Bad trajectory input!!");
            e.printStackTrace();
            return new Trajectory100();
        }
    }
}
