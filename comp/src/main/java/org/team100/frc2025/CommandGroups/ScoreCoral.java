// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import org.team100.frc2025.FieldConstants.FieldSector;
import org.team100.frc2025.FieldConstants.ReefDestination;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Swerve.SemiAuto.Profile_Nav.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoral extends SequentialCommandGroup {
  /** Creates a new ScoreLevel. */
  public ScoreCoral(LoggerFactory logger, Wrist2 wrist, Elevator elevator, CoralTunnel tunnel, FieldSector targetSector, ReefDestination destination, Supplier<ScoringPosition> scoringPositionSupplier,  SwerveController controller, HolonomicProfile profile, SwerveDriveSubsystem m_drive) {


    addCommands(
        new Embark(m_drive, SwerveControllerFactory.byIdentity(logger), profile, FieldSector.AB, ReefDestination.CENTER),
        new SelectSequence(
            scoringPositionSupplier,
            new ScoreL1(),
            new ScoreL2(),
            new ScoreL3(),
            new ScoreL4(),
            wrist,
            elevator)




    );








    //L4
    // addCommands(
    //     new SetWrist(wrist, 0.5, false),
    //     new ParallelDeadlineGroup(new SetElevator(elevator, 45, false), new SetWrist(wrist, 0.5, true)), //45
    //     new ParallelDeadlineGroup(new SetWrist(wrist, 1.25, false), new SetElevatorPerpetually(elevator, 45)),
    //     new ParallelDeadlineGroup(new SetElevator(elevator, 35, false), new SetWrist(wrist, 1.25, true)),
    //     new ParallelDeadlineGroup(new WaitCommand(10), new SetWristDutyCycle(wrist, -1), new SetElevatorPerpetually(elevator, 35))
    //     // new ParallelDeadlineGroup(new SetWrist(wrist, 0.5, false), new SetElevatorPerpetually(elevator, 35)), 
    //     // new SetElevator(elevator, 2, false)

    //     // new SetWrist(wrist, 0.5, false),

    //     // new ParallelDeadlineGroup(new SetElevator(elevator, 10, false), new SetWrist(wrist, 1.26, false))
    //     // new SetWrist(wrist, 0.5, false)

    // );

    // addCommands(
    //     new SetWrist(wrist, 0.5, false),
    //     new ParallelDeadlineGroup(new SetElevator(elevator, 23, false), new SetWrist(wrist, 0.5, true)), //45
    //     new ParallelDeadlineGroup(new SetWrist(wrist, 1.25, false), new SetElevatorPerpetually(elevator, 23)),
    //     new ParallelDeadlineGroup(new SetElevator(elevator, 12, false), new SetWrist(wrist, 1.25, true)),
    //     new ParallelDeadlineGroup(new SetWrist(wrist, 0.5, false), new SetElevatorPerpetually(elevator, 12)),
    //     new ParallelDeadlineGroup(new SetElevator(elevator, 2, false), new SetWrist(wrist, 0.5, true))
    //     // new ParallelDeadlineGroup(new WaitCommand(10), new SetWristDutyCycle(wrist, -1), new SetElevatorPerpetually(elevator, 35))
        

    // );

    // addCommands(
        // new SetWrist(wrist, 0.5, false),
    //     new ParallelDeadlineGroup(new SetElevator(elevator, 11, false), new SetWrist(wrist, 0.5, true)), //45
    //     new ParallelDeadlineGroup(new SetWrist(wrist, 1.25, false), new SetElevatorPerpetually(elevator, 11)),
    //     new ParallelDeadlineGroup(new SetElevator(elevator, 3.6, false), new SetWrist(wrist, 1.25, true))
    //     // new ParallelDeadlineGroup(new SetWrist(wrist, 0.5, false), new SetElevatorPerpetually(elevator, 6)),
    //     // new ParallelDeadlineGroup(new SetElevator(elevator, 2, false), new SetWrist(wrist, 0.5, true))
    //     // new ParallelDeadlineGroup(new WaitCommand(10), new SetWristDutyCycle(wrist, -1), new SetElevatorPerpetually(elevator, 35))
        

    // );

    // addCommands(
    //     // new SetWrist(wrist, 2.8, false),
    //     new SetWrist(wrist, 0.5, false),
    //     new SetElevator(elevator, 9.5, false),
    //     new ParallelDeadlineGroup(new SetWrist(wrist, 2.387268, false), new SetElevatorPerpetually(elevator, 9)),
    //     new ParallelDeadlineGroup(new RunCoralTunnel(tunnel, -1), new SetElevatorPerpetually(elevator, 8.1))
    //     // new ParallelDeadlineGroup(new SetElevator(elevator, 3.6, false), new SetWrist(wrist, 1.25, true))
    //     // new ParallelDeadlineGroup(new SetWrist(wrist, 0.5, false), new SetElevatorPerpetually(elevator, 6)),
    //     // new ParallelDeadlineGroup(new SetElevator(elevator, 2, false), new SetWrist(wrist, 0.5, true))
    //     // new ParallelDeadlineGroup(new WaitCommand(10), new SetWristDutyCycle(wrist, -1), new SetElevatorPerpetually(elevator, 35))
        

    // );
  }
}
