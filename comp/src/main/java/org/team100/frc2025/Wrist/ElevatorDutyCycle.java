
package org.team100.frc2025.Wrist;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDutyCycle extends Command {
    Elevator m_elevator;

    public ElevatorDutyCycle(Elevator elevator) {

        m_elevator = elevator;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setDutyCycle(-0.03);
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("I FINISHED NUMBER 2");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
