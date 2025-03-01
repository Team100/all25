
package org.team100.frc2025.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDown extends Command {
    /** Creates a new SetElevator. */
    Elevator m_elevator;

    public ElevatorDown(Elevator elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        // m_elevator.resetProfile();
    }

    @Override
    public void execute() {
        // m_elevator.setDutyCycle(-0.05);
        m_elevator.setPosition(6); //16 for l3
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
