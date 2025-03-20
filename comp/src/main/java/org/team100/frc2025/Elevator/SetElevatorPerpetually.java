package org.team100.frc2025.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorPerpetually extends Command {
    Elevator m_elevator;
    double m_value;
    boolean finished = false;

    public SetElevatorPerpetually(Elevator elevator, double value) {
        m_elevator = elevator;
        m_value = value;
        finished = false;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.resetElevatorProfile();
        finished = false;
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_value); // 24.5 for l3

        // double error = Math.abs(m_elevator.getPosition() - m_value);
        // if(error < 0.5){
        // finished = true;
        // }
        // m_elevator.setDutyCycle(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        finished = false;

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
