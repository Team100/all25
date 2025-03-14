// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeAlgaeGrip extends Command {
  /** Creates a new RunAlgaeManipulator. */
  AlgaeGrip m_grip;


  public OuttakeAlgaeGrip(AlgaeGrip tunnel) {
    m_grip = tunnel;
    addRequirements(m_grip);
  }

  @Override
  public void initialize() {
    m_grip.setDutyCycle(-1);
  }

  @Override
  public void execute() {
    // m_grip.intake();
  }

  public void end(boolean interrupted) {
    m_grip.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
