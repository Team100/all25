// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgaeGrip extends Command {
  /** Creates a new RunAlgaeManipulator. */
  GripState m_gripState;
  AlgaeGrip m_grip;

  public enum GripState {
    INTAKE,
    OUTAKE,
    STOP
  }

  public RunAlgaeGrip(AlgaeGrip tunnel, GripState gripState) {
    m_grip = tunnel;
    m_gripState = gripState;
    addRequirements(m_grip);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (m_gripState) {
        case INTAKE:
            m_grip.intake();
            break;
        case OUTAKE:
            m_grip.outtake();
            break;
        case STOP:
            m_grip.stop();
            break;
    }    
  }

  public void end(boolean interrupted) {
    m_grip.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
