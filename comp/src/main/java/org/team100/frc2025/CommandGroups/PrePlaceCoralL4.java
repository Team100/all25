package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class PrePlaceCoralL4 extends Command {
    Wrist2 m_wrist;
    Elevator m_elevator;
    double m_elevatorGoal;
    double count = 0;
    boolean finished = false;

    public PrePlaceCoralL4(Wrist2 wrist, Elevator elevator, double elevatorValue) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        addRequirements(m_wrist, m_elevator);
    }

    @Override
    public void initialize() {
        count = 0;
        finished = false;
        m_wrist.resetWristProfile();
        m_elevator.resetElevatorProfile();
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_elevatorGoal);
        if (m_elevatorGoal - 10 > m_elevator.getPosition()) {
            m_wrist.setAngleValue(0.4);
        } else {
            m_wrist.setAngleValue(1.25);
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

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
