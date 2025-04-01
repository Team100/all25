// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.CommandGroups.PrePlaceCoralL2;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL2Smart extends SequentialCommandGroup {
  /** Creates a new ScoreL2Smart. */
  public ScoreL2Smart(LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> height,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            ReefPoint reefPoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup100(logger, "drive",
                        new Embark(logger, m_drive, heedRadiusM, controller, profile, targetSector, destination, height, reefPoint),
                        new SequentialCommandGroup100(logger, "out",
                                // new WaitUntilWithinRadius(m_drive),
                                new SetWrist(wrist, 0.4, false),
                                new PrePlaceCoralL2(wrist, elevator, 10.5, true))),
        new ParallelDeadlineGroup100(logger, "score",
                        new SetElevator(logger, elevator, 10.5, false),
                        new SetWrist(wrist, 0.55, true)),
        new ParallelDeadlineGroup100(logger, "down",
                        new SetElevator(logger, elevator, 1.5, false),
                        new SetWrist(wrist, 0.65, true))
    );
  }
}
