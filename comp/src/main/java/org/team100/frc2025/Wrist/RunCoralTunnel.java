// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCoralTunnel extends Command {
  /** Creates a new RunAlgaeManipulator. */
  double m_value;
  CoralTunnel m_tunnel;
  public RunCoralTunnel(CoralTunnel tunnel, double value) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tunnel = tunnel;
    m_value = value;
    addRequirements(m_tunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_wrist.setAlgaeMotor(0.5);
    m_tunnel.setCoralMotor(m_value);

    // System.out.println("Coral Tunnel Motor MOOOVING");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_wrist.setCoralMotor(0);
    m_tunnel.setCoralMotor(0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
