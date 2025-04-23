package org.team100.frc2025.CommandGroups.ScoreSmart;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorAndWrist extends Command {
    Elevator m_elevator;
    Wrist2 m_wrist;
    double m_elevatorValue;
    double m_wristValue;
    double countElevator = 0;
    double countWrist = 0;
    boolean finished = false;

    public SetElevatorAndWrist(Elevator elevator, Wrist2 wrist, double elevatorValue, double wristValue) {
        m_elevator = elevator;
        m_wrist = wrist;

        m_elevatorValue = elevatorValue;
        m_wristValue = wristValue;
        addRequirements(m_elevator, m_wrist);
    }

    @Override
    public void initialize() {
        countElevator = 0;
        countWrist = 0;
        finished = false;
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_elevatorValue);
        m_wrist.setAngleValue(m_wristValue);

        double errorElevator = Math.abs(m_elevator.getPosition() - m_elevatorValue);
        double errorWrist = Math.abs(m_wrist.getAngle() - m_wristValue);

        if (errorElevator < 0.5) {
            countElevator++;
        } else {
            countElevator = 0;
        }

        if (errorWrist < 0.05) {
            countWrist++;
        } else {
            countWrist = 0;
        }

        if (countElevator >= 2 && countWrist >= 1) {
            finished = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return finished && m_wrist.profileDone() && m_elevator.profileDone();
    }
}
