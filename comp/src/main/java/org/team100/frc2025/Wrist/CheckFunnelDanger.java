package org.team100.frc2025.Wrist;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class CheckFunnelDanger extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private double m_initialElevatorValue;
    private boolean finished = false;

    public CheckFunnelDanger(Wrist2 wrist, Elevator elevator) {
        m_wrist = wrist;
        m_elevator = elevator;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_initialElevatorValue = m_elevator.getPosition();
        finished = false;

    }

    @Override
    public void execute() {
        if (m_wrist.getAngle() > 1.6) {
            m_wrist.setAngleValue(1.5);
            m_elevator.setPosition(m_initialElevatorValue);
        } else {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
