package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.List;
import java.util.function.DoubleConsumer;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.commands.drivetrain.DriveToTranslationFacingWithProfile;
import org.team100.lib.commands.drivetrain.DriveWithTrajectoryFunction;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class LolipopAuto {
    private static final double HEED_RADIUS_M = 3;

    public static Command get(
            LoggerFactory logger,
            CalgamesMech mech,
            Manipulator manipulator,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem drive,
            TrajectoryPlanner planner,
            DoubleConsumer heedRadiusM,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {

        // this one uses some curvature
        DriveWithTrajectoryFunction toReefTrajectory = new DriveWithTrajectoryFunction(
                drive, controller, viz,
                (p) -> planner.restToRest(List.of(
                        HolonomicPose2d.make(drive.getPose(), Math.PI),
                        HolonomicPose2d.make(3, 5, 0, -2))));

        DriveToPoseWithProfile toReefA = new DriveToPoseWithProfile(
                logger, drive, controller, profile,
                () -> FieldConstants.makeGoal(ScoringLevel.L4, ReefPoint.A));

        DriveToTranslationFacingWithProfile toCenterCoral = new DriveToTranslationFacingWithProfile(
                logger, drive, controller, profile,
                () -> FieldConstants.CoralMark.CENTER.value
                        .plus(new Translation2d(0.7, 0)),
                new Rotation2d(Math.PI));

        DriveToPoseWithProfile toReefB = new DriveToPoseWithProfile(
                logger, drive, controller, profile,
                () -> FieldConstants.makeGoal(ScoringLevel.L4, ReefPoint.B));

        DriveToTranslationFacingWithProfile toCoralRight = new DriveToTranslationFacingWithProfile(
                logger, drive, controller, profile,
                () -> FieldConstants.CoralMark.RIGHT.value
                        .plus(new Translation2d(0.7, 0).rotateBy(new Rotation2d(Math.PI / 4))),
                new Rotation2d(Math.PI));

        DriveToPoseWithProfile toReefC = new DriveToPoseWithProfile(
                logger, drive, controller, profile,
                () -> FieldConstants.makeGoal(ScoringLevel.L4, ReefPoint.C));

        MoveAndHold toL4 = mech.homeToL4();
        MoveAndHold toL4second = mech.homeToL4();
        MoveAndHold toL4third = mech.homeToL4();
        ParallelRaceGroup eject = manipulator.centerEject().withTimeout(0.3);
        ParallelRaceGroup ejectsecond = manipulator.centerEject().withTimeout(0.3);
        ParallelRaceGroup ejectthird = manipulator.centerEject().withTimeout(0.3);
        return sequence(
                toReefTrajectory.until(toReefTrajectory::isDone),
                parallel(
                        runOnce(() -> heedRadiusM.accept(HEED_RADIUS_M)),
                        toReefA,
                        mech.profileHomeAndThenRest().until(toReefA::isDone).andThen(toL4),
                        waitUntil(() -> toReefA.isDone() && toL4.isDone())
                                .andThen(eject))
                        .until(() -> (toReefA.isDone() && toL4.isDone() && eject.isFinished())),
                parallel(
                        toCenterCoral,
                        mech.pickWithProfile(),
                        manipulator.centerIntake()).until(manipulator::hasCoral).withTimeout(3)
                        .andThen(manipulator::stop),
                parallel(
                        toReefB,
                        mech.profileHomeAndThenRest().until(toReefB::isDone).andThen(toL4second),
                        waitUntil(() -> toReefB.isDone() && toL4second.isDone())
                                .andThen(ejectsecond))
                        .until(() -> (toReefB.isDone() && toL4second.isDone() && ejectsecond.isFinished())),
                parallel(
                        toCoralRight,
                        mech.pickWithProfile(),
                        manipulator.centerIntake()).until(manipulator::hasCoral).withTimeout(3)
                        .andThen(manipulator::stop),
                parallel(
                        toReefC,
                        mech.profileHomeAndThenRest().until(toReefC::isDone).andThen(toL4third),
                        waitUntil(() -> toReefC.isDone() && toL4third.isDone())
                                .andThen(ejectthird))
                        .until(() -> (toReefC.isDone() && toL4third.isDone() && ejectthird.isFinished())),
                mech.l4ToHome());
    }
}
