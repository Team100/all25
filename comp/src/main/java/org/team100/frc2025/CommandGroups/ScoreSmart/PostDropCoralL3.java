// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups.ScoreSmart;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PostDropCoralL3 extends Command {
  /** Creates a new PostDropCoralL3. */
  private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final double m_elevatorGoal;

    private double count = 0;
    private boolean finished = false;
    private double initialElevatorPosition = 0;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final Command m_holdingCommand;

    public PostDropCoralL3(Wrist2 wrist, Elevator elevator, double elevatorValue, Command holdingCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        m_holdingCommand = holdingCommand;
        addRequirements(m_wrist, m_elevator);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

        count = 0;
        finished = false;
        // resetting forces the setpoint velocity to zero, which is not always what we
        // want
        // m_wrist.resetWristProfile();
        // m_elevator.resetElevatorProfile();
        if(m_holdingCommand != null){
            CommandScheduler.getInstance().cancel(m_holdingCommand);
        }
        initialElevatorPosition = m_elevator.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setPosition(m_elevatorGoal);

        if (Math.abs(m_elevator.getPosition() - initialElevatorPosition) > 0.5) {
            m_wrist.setAngleValue(0.4);
        } else {
            m_wrist.setAngleValue(0.9);
        }

        double error = Math.abs(m_elevator.getPosition() - m_elevatorGoal);

        if (error < 0.5) {
            count++;
        } else {
            count = 0;
        }

        if (count >= 5) {
            finished = true;
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
