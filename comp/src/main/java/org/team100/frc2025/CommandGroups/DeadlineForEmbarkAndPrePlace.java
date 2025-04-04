// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import org.team100.lib.util.Takt;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeadlineForEmbarkAndPrePlace extends Command {
  /** Creates a new DeadlineForEmbarkAndPrePlace. */
  Supplier<Boolean> m_embarkEnd;
  Supplier<Boolean> m_prePlaceEnd;

  boolean finished = false;
  public DeadlineForEmbarkAndPrePlace(Supplier<Boolean> embarkEnd, Supplier<Boolean> prePlaceEnd) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_embarkEnd = embarkEnd;
    m_prePlaceEnd = prePlaceEnd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    
    if(m_embarkEnd.get() && m_prePlaceEnd.get()){
        finished = true;
    } else {
        finished = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_embarkEnd.get() && m_prePlaceEnd.get()){
        finished = true;
    } else {
        finished = false;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("DEADLINE FINISHED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
