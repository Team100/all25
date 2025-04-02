// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForBooleanTrue extends Command {
  /** Creates a new WaitForBooleanTrue. */
  Supplier<Boolean> m_booleanSupplier;
  boolean finished = false;
  public WaitForBooleanTrue(Supplier<Boolean> booleanSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_booleanSupplier = booleanSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_booleanSupplier.get()){
        finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(" I THINK YOURE TRUE ");
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
