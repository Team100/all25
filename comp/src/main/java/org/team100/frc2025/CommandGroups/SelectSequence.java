// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SelectSequence extends Command {

  Command m_l1Command;
  Command m_l2Commad;
  Command m_l3Command;
  Command m_l4Command;
  Wrist2 m_wrist;
  Elevator m_elevator;
  Supplier<ScoringPosition> m_scoringSupplier;
  ScoringPosition m_position = ScoringPosition.NONE;
  /** Creates a new SelectSequence. */
  public SelectSequence(Supplier<ScoringPosition> scoringSupplier, Command l1Command, Command l2Commad, Command l3Command, Command l4Command, Wrist2 wrist, Elevator elevator) {

    m_l1Command = l1Command;
    m_l2Commad = l2Commad;
    m_l3Command = l3Command;
    m_l4Command = l4Command;
    m_wrist = wrist;
    m_elevator = elevator;
    m_scoringSupplier = scoringSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_position = m_scoringSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_position){
      case L4:
        m_l4Command.execute();
        break;
      case L3:
        m_l3Command.execute();
        break;
      case L2:
        m_l2Commad.execute();
        break;
      case L1:
        m_l1Command.execute();
        break;
      case NONE:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
