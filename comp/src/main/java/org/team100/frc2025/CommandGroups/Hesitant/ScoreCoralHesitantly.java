// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups.Hesitant;

import java.util.Map;
import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.CommandGroups.ScoreL1;
import org.team100.frc2025.CommandGroups.ScoreL2;
import org.team100.frc2025.CommandGroups.ScoreL3;
import org.team100.frc2025.CommandGroups.ScoreL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.manual.DriveAdjustCoral;
import org.team100.lib.commands.drivetrain.manual.DriveManually;
import org.team100.lib.commands.drivetrain.manual.FieldRelativeDriver;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoralHesitantly extends SequentialCommandGroup {
  /** Creates a new ScoreCoralHesitantly. */
  public ScoreCoralHesitantly(LoggerFactory logger, Wrist2 wrist, Elevator elevator, CoralTunnel tunnel, FieldSector targetSector, ReefDestination destination, Supplier<ScoringPosition> scoringPositionSupplier,  SwerveController controller, HolonomicProfile profile, SwerveDriveSubsystem m_drive, Supplier<DriverControl.Velocity> twistSupplier, Supplier<Boolean> readyToPlace, FieldRelativeDriver driver ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetElevatorTargetPosition(elevator, scoringPositionSupplier),
        new EmbarkHesitantly(m_drive, SwerveControllerFactory.byIdentity(logger), profile, targetSector, destination, elevator),

        new ParallelDeadlineGroup(
            new DriveAdjustCoral(twistSupplier, m_drive, readyToPlace, driver),
            new SelectCommand<>(
                Map.of(
                    ScoringPosition.L4, new PreScoreL4Hesitant(wrist, elevator)
                )
                ,elevator::getScoringPosition
            )
        ),
        new SelectCommand<>(
                Map.of(
                    ScoringPosition.L4, new ScoreL4Hesitantly(wrist, elevator)
                )
                ,elevator::getScoringPosition
        )
        
        
        
    );
  }
}
