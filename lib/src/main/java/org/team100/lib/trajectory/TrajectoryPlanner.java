package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathFactory;
import org.team100.lib.timing.ScheduleGenerator;
import org.team100.lib.timing.TimingConstraint;
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
    // the tolerances here control the approximation of the list of poses to the
    // continuous spline. previously they were 1.2 cm and 1 radian.
    private static final double kSplineSampleToleranceM = 0.05;
    private static final double kSplineSampleToleranceRad = 0.2;
    private static final double kTrajectoryStepM = 0.1;
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

    public Trajectory100 restToRest(List<HolonomicPose2d> waypoints) {
        return generateTrajectory(waypoints, 0.0, 0.0);
    }

    public Trajectory100 restToRest(List<HolonomicPose2d> waypoints, List<Double> mN) {
        return generateTrajectory(waypoints, 0.0, 0.0, mN);
    }

    public Trajectory100 movingToRest(SwerveModel startState, Pose2d end) {
        return movingToMoving(startState, new SwerveModel(end));
    }

    public Trajectory100 movingToMoving(SwerveModel startState, SwerveModel endState) {
        Translation2d startTranslation = startState.translation();
        FieldRelativeVelocity startVelocity = startState.velocity();

        Translation2d endTranslation = endState.translation();
        FieldRelativeVelocity endVelocity = endState.velocity();

        // should work with out this.
        if (startVelocity.norm() < VELOCITY_EPSILON && endVelocity.norm() < VELOCITY_EPSILON) {
            return restToRest(startState.pose(), endState.pose());
        }

        Translation2d full = endTranslation.minus(startTranslation);
        Rotation2d courseToGoal = full.getAngle();
        Rotation2d startingAngle = startVelocity.angle().orElse(courseToGoal);

        // use the start velocity to adjust the first magic number.
        // divide by the distance because the spline multiplies by it
        double e1 = 2.0 * startVelocity.norm() / full.getNorm();
        List<Double> magicNumbers = List.of(e1, 1.2);

        try {
            return generateTrajectory(
                    List.of(
                            new HolonomicPose2d(
                                    startTranslation,
                                    startState.rotation(),
                                    startingAngle),
                            new HolonomicPose2d(
                                    endTranslation,
                                    endState.rotation(),
                                    courseToGoal)),
                    startVelocity.norm(),
                    endVelocity.norm(),
                    magicNumbers);
        } catch (TrajectoryGenerationException e) {
            Util.warn("Trajectory Generation Exception");
            return new Trajectory100();
        }
    }

    /**
     * Produces straight lines from start to end.
     */
    public Trajectory100 restToRest(Pose2d start, Pose2d end) {
        Translation2d startTranslation = start.getTranslation();
        Translation2d endTranslation = end.getTranslation();

        Rotation2d courseToGoal = endTranslation.minus(startTranslation).getAngle();

        try {
            return restToRest(
                    List.of(
                            new HolonomicPose2d(startTranslation, start.getRotation(), courseToGoal),
                            new HolonomicPose2d(endTranslation, end.getRotation(), courseToGoal)));
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }

    /**
     * The shape of the spline accommodates the start and end velocities.
     */
    public Trajectory100 generateTrajectory(
            List<HolonomicPose2d> waypoints,
            double start_vel,
            double end_vel) {
        try {
            // Create a path from splines.
            Path100 path = PathFactory.pathFromWaypoints(
                    waypoints,
                    kSplineSampleToleranceM,
                    kSplineSampleToleranceM,
                    kSplineSampleToleranceRad);
            // Generate the timed trajectory.
            return m_scheduleGenerator.timeParameterizeTrajectory(
                    path,
                    kTrajectoryStepM,
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
            List<HolonomicPose2d> waypoints,
            double start_vel,
            double end_vel,
            List<Double> mN) {
        try {
            // Create a path from splines.
            Path100 path = PathFactory.pathFromWaypoints(
                    waypoints,
                    kSplineSampleToleranceM,
                    kSplineSampleToleranceM,
                    kSplineSampleToleranceRad, mN);
            // Generate the timed trajectory.
            return m_scheduleGenerator.timeParameterizeTrajectory(
                    path,
                    kTrajectoryStepM,
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
