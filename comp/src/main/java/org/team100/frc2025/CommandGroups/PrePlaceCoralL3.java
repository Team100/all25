package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;

public class PrePlaceCoralL3 extends Command {
    Wrist2 m_wrist;
    Elevator m_elevator;
    double m_elevatorGoal;
    double count = 0;
    boolean finished = false;
    boolean m_perpetual = false;
    CoralTunnel m_tunnel;

    public PrePlaceCoralL3(Wrist2 wrist, Elevator elevator, CoralTunnel tunnel, double elevatorValue, boolean perpetual) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        m_perpetual = perpetual;
        m_tunnel = tunnel;
        addRequirements(m_wrist, m_elevator, m_tunnel);
    }

    @Override
    public void initialize() {
        count = 0;
        finished = false;
        // resetting forces the setpoint velocity to zero, which is not always what we
        // want
        // m_wrist.resetWristProfile();
        // m_elevator.resetElevatorProfile();
    }

    @Override
    public void execute() {
        m_tunnel.setCoralMotor(1);
        m_elevator.setPosition(m_elevatorGoal);
        if (m_elevatorGoal - 10 > m_elevator.getPosition()) {
            m_wrist.setAngleValue(0.4);
        } else {
            m_wrist.setAngleValue(0.9);
        }

        double error = Math.abs(m_elevator.getPosition() - m_elevatorGoal);

        if (error < 0.5) {
            count++;
        } else {
            count = 0;
        }

        if (count >= 10) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_tunnel.setCoralMotor(0);

    }

    public boolean isDone() {
        if (Experiments.instance.enabled(Experiment.UseProfileDone)){
            return finished && m_wrist.profileDone() && m_elevator.profileDone();
        }
        return finished;
        
    }

    @Override
    public boolean isFinished() {
        if(!m_perpetual){
            if (Experiments.instance.enabled(Experiment.UseProfileDone)){
                return finished && m_wrist.profileDone() && m_elevator.profileDone();
            }
            return finished;
        } else {
            return false;
        }
        
    }
}
