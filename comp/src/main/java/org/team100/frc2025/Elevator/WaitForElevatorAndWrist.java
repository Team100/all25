package org.team100.frc2025.Elevator;

import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForElevatorAndWrist extends Command {
    Elevator m_elevator;
    Wrist2 m_wrist;
    boolean finished = false;

    public WaitForElevatorAndWrist(Elevator elevator, Wrist2 wrist) {
        m_elevator = elevator;
        m_wrist = wrist;

    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        if (m_elevator.getPosition() <= 20 && m_wrist.getAngle() <= 0.9) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
