// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Swerve.WaitUntilWithinRadius;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL4Smart extends SequentialCommandGroup {
  /** Creates a new ScoreL1Smart. */
  public ScoreL4Smart( LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> scoringPositionSupplier,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(
            new Embark(
                m_drive,
                heedRadiusM,
                SwerveControllerFactory.byIdentity(logger),
                profile,
                targetSector,
                destination,
                scoringPositionSupplier),
            new SequentialCommandGroup(
                new WaitUntilWithinRadius(m_drive),
                new SetWrist(wrist, 0.4, false),
                new PrePlaceCoralL4(wrist, elevator, 45)
            )
        ),
        new ParallelDeadlineGroup100(
                logger,
                new SetWrist(wrist, 1.25, false),
                new SetElevatorPerpetually(elevator, 45)
        ),
        new ParallelDeadlineGroup100(logger,
                new SetElevator(elevator, 35, false),
                new SetWrist(wrist, 1.25, true)
        ),
        new ParallelDeadlineGroup100(logger,
                new SetElevator(elevator, 10, false),
                new SetWrist(wrist, 0.5, true)
        )
            
    );
  }
}
