package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static org.team100.lib.config.ElevatorUtil.ScoringLevel.L4;
import static org.team100.lib.field.FieldConstants.ReefPoint.C;
import static org.team100.lib.field.FieldConstants.ReefPoint.D;
import static org.team100.lib.field.FieldConstants.ReefPoint.F;
import static org.team100.lib.field.FieldConstants.ReefPoint.I;
import static org.team100.lib.field.FieldConstants.ReefPoint.H;
import static org.team100.lib.field.FieldConstants.ReefPoint.K;
import static org.team100.lib.field.FieldConstants.ReefPoint.L;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.commands.drivetrain.DriveWithTrajectoryFunction;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.CoralStation;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

// it's a record to make it less verbose
public record Auton(LoggerFactory logger, CalgamesMech mech,
        Manipulator manipulator,
        SwerveController controller, HolonomicProfile profile,
        SwerveDriveSubsystem drive,
        DoubleConsumer heedRadiusM, SwerveKinodynamics kinodynamics,
        TrajectoryVisualization viz) {

    static final boolean AUTON_FIXED = false;

    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double HEED_RADIUS_M = 3;

    public Command leftPreloadOnly() {
        return sequence(
                embarkAndPreplace(L4, I),
                manipulator.centerEject().withTimeout(0.5),
                mech.l4ToHome());
    }

    public Command centerPreloadOnly() {
        return sequence(
                embarkAndPreplace(L4, H),
                manipulator.centerEject().withTimeout(0.5),
                mech.l4ToHome());
    }

    public Command rightPreloadOnly() {
        return sequence(
                embarkAndPreplace(L4, F),
                manipulator.centerEject().withTimeout(0.5),
                mech.l4ToHome());
    }

    public Command left() {
        if (!AUTON_FIXED)
            return print("LEFT AUTON: do not use until it is adjusted for the new pick location\n");
        return sequence(
                embarkAndPreplace(L4, I),
                scoreAndReload(CoralStation.LEFT),
                embarkAndPreplace(L4, K),
                scoreAndReload(CoralStation.LEFT),
                embarkAndPreplace(L4, L),
                manipulator.centerEject().withTimeout(0.5),
                mech.l4ToHome());
    }

    public Command right() {
        if (!AUTON_FIXED)
            return print("RIGHT AUTON: do not use until it is adjusted for the new pick location\n");
        return sequence(
                embarkAndPreplace(L4, F),
                scoreAndReload(CoralStation.RIGHT),
                embarkAndPreplace(L4, D),
                scoreAndReload(CoralStation.RIGHT),
                embarkAndPreplace(L4, C),
                manipulator.centerEject().withTimeout(0.5),
                mech.l4ToHome());
    }

    /** Drive to the reef and go up. */
    private Command embarkAndPreplace(ScoringLevel position, ReefPoint point) {
        DriveToPoseWithProfile toReef = new DriveToPoseWithProfile(
                logger, drive, controller, profile,
                () -> FieldConstants.makeGoal(position, point));
        MoveAndHold toL4 = mech.homeToL4();
        return parallel(
                runOnce(() -> heedRadiusM.accept(HEED_RADIUS_M)),
                toReef,
                waitUntil(() -> toReef.toGo() < 1)
                        .andThen(toL4) //
        ).until(() -> (toReef.isDone() && toL4.isDone()));
    }

    /** Score, drive to the station, and pause briefly. */
    private Command scoreAndReload(CoralStation station) {
        GoToCoralStation toStation = new GoToCoralStation(kinodynamics, station, 0.5);
        DriveWithTrajectoryFunction navigator = new DriveWithTrajectoryFunction(drive, controller, viz, toStation);
        return sequence(
                // first fire the coral at the peg
                manipulator.centerEject()
                        .withTimeout(0.5),
                parallel(
                        // then stow the arm
                        mech.l4ToHome(),
                        sequence(
                                // while the arm is still in motion
                                // wait for it to be low enough
                                waitUntil(mech::isSafeToDrive),
                                // and then drive to pick
                                navigator) //
                ).until(navigator::isDone),
                // then move the arm into the station and run the intake.
                parallel(
                        mech.stationWithProfile(),
                        manipulator.centerIntake() //
                ).until(manipulator::hasCoral) //
        );
    }
}
