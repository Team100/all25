package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import java.util.List;

import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.field.MechanicalMayhem2025;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.subsystems.tank.commands.FixedTrajectory;
import org.team100.lib.subsystems.tank.commands.ToPoseWithTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The auton selector is just a flat list.
 * 
 * Each item contains:
 * 
 * * a command
 * * a starting position
 * * an alliance color
 * 
 * These should be checked against the true color and true position, and an
 * alert should be raised if they're wrong.
 * 
 * If a command works for arbitrary position, leave it null.
 * 
 * If a command works for either alliance, leave it null.
 * 
 * The name should be fully descriptive, i.e. contain something about the
 * position and color if that's required to choose it in the dashboard.
 */
public class Autons {
    private final LoggerFactory m_log;
    private final TankDrive m_drive;
    private final TrajectoryVisualization m_trajectoryViz;
    private final AutonChooser m_autonChooser;

    public Autons(
            LoggerFactory log,
            TankDrive drive,
            TrajectoryVisualization trajectoryViz) {
        m_log = log.type(this);
        m_drive = drive;
        m_trajectoryViz = trajectoryViz;

        m_autonChooser = new AutonChooser();

        m_autonChooser.add("red left",
                new AnnotatedCommand(redLeft(), Alliance.Red, Field.START_RED_LEFT));

        m_autonChooser.add("blue left",
                new AnnotatedCommand(blueLeft(), Alliance.Blue, Field.START_BLUE_LEFT));

        m_autonChooser.add("red center",
                new AnnotatedCommand(redCenter(), Alliance.Red, Field.START_RED_CENTER));

        m_autonChooser.add("blue center",
                new AnnotatedCommand(blueCenter, Alliance.Blue, Field.START_BLUE_CENTER));

        m_autonChooser.add("red right",
                new AnnotatedCommand(redRight(), Alliance.Red, Field.START_RED_RIGHT));

        m_autonChooser.add("blue right",
                new AnnotatedCommand(blueRight, Alliance.Blue, Field.START_BLUE_RIGHT));

    }

    public AnnotatedCommand get() {
        return m_autonChooser.get();
    }



    private Trajectory100 redLeftTrajectory(TrajectoryPlanner planner) {
        // delaying construction allows trajectory constraints to be mutable
        return planner.restToRest(List.of(
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_LEFT),
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_LEFT
                        .plus(new Transform2d(1, 1, Rotation2d.kCCW_90deg))),
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_LEFT
                        .plus(new Transform2d(2, 2, Rotation2d.kZero)))));
    }

    private Command redRight() {
        LoggerFactory log = m_log.name("red right");
        ToPoseWithTrajectory cmd = new ToPoseWithTrajectory(
                log,
                Field.START_RED_RIGHT
                        .plus(new Transform2d(1, 1, Rotation2d.kCW_90deg)),
                m_drive,
                m_trajectoryViz);
        return cmd.until(cmd::isDone).withName("red right");
    }
}

    private Command redLeft() {
        LoggerFactory log = m_log.name("red left");
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(log, 1, 1)));
        FixedTrajectory cmd = new FixedTrajectory(
                () -> redLeftTrajectory(planner),
                m_drive,
                m_trajectoryViz);
        return cmd.until(cmd::isDone).withName("red left");
    }