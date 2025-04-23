package org.team100.frc2025.Elevator;

import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class HoldWristAndElevator extends Command {
    Elevator m_elevator;
    Wrist2 m_wrist;

    double initialElevatorValue;
    double initialWristValue;

    public HoldWristAndElevator(Elevator elevator, Wrist2 wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
        addRequirements(m_elevator, m_wrist);
    }

    @Override
    public void initialize() {
        System.out.println("I AM STARTING RIGHT NOW");

        initialElevatorValue = m_elevator.getPosition();
        initialWristValue = m_wrist.getAngle();
    }

    @Override
    public void execute() {
        m_elevator.setPosition(initialElevatorValue);
        m_wrist.setAngleValue(initialWristValue);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("I AM FINISHING RIGHT NOW");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
