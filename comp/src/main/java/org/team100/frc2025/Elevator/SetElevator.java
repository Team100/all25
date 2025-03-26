package org.team100.frc2025.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevator extends Command {
    Elevator m_elevator;
    double m_value;
    boolean finished = false;
    boolean m_perpetual;
    double count = 0;

    public SetElevator(Elevator elevator, double value, boolean perpetual) {
        m_elevator = elevator;
        m_value = value;
        finished = false;
        m_perpetual = perpetual;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {

        count = 0;
        finished = false;
        m_elevator.resetElevatorProfile();
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_value); // 24.5 for l3

        double error = Math.abs(m_elevator.getPosition() - m_value);
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
        m_elevator.stop();
        finished = false;
        count = 0;

    }

    @Override
    public boolean isFinished() {
        if (m_perpetual) {
            return false;
        } else {
            return finished;
        }
    }
}
