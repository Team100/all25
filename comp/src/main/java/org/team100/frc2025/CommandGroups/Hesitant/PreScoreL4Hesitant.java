// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups.Hesitant;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PreScoreL4Hesitant extends SequentialCommandGroup {
    public PreScoreL4Hesitant(LoggerFactory parent, Wrist2 wrist, Elevator elevator) {

        addCommands(
                new SetWrist(wrist, 0.4, false),
                new ParallelDeadlineGroup100(parent,
                        new SetElevator(elevator, 45, false),
                        new SetWrist(wrist, 0.4, true)), // 45
                new ParallelDeadlineGroup100(parent,
                        new SetWrist(wrist, 1.25, true),
                        new SetElevatorPerpetually(elevator, 45)));
    }
}
