// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups.Hesitant;

import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorTargetPosition extends Command {
  /** Creates a new SetElevatorTargetPosition. */
  Elevator m_elevator;
  Supplier<ScoringPosition> positionSupplier;
  public SetElevatorTargetPosition(Elevator elevator, Supplier<ScoringPosition> pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    positionSupplier = pos;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setTargetScoringPosition(positionSupplier.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
