package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorFunnelHandoff extends Command {
    Elevator m_elevator;
    double m_value;
    boolean finished = false;
    double count = 0;

    public SetElevatorFunnelHandoff(Elevator elevator, double value) {
        m_elevator = elevator;
        m_value = value;
        finished = false;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        count = 0;
        finished = false;
        // resetting forces the setpoint velocity to zero, which is not always what we
        // want
        // m_elevator.resetElevatorProfile();
    }

    @Override
    public void execute() {
        m_elevator.setPositionNoGravity(m_value); // 24.5 for l3

        double error = Math.abs(m_elevator.getPosition() - m_value);
        if (error < 1) {
            count++;
        } else {
            count = 0;
        }

        if (count >= 2) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        finished = false;
        count = 0;
        // System.out.println("I FINISHED NUMBER 1");

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
