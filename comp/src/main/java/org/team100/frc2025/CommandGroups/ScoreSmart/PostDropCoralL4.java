package org.team100.frc2025.CommandGroups.ScoreSmart;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PostDropCoralL4 extends Command {
    private final Wrist2 m_wrist;
    private final Elevator m_elevator;
    private final double m_elevatorGoal;

    private double count = 0;
    private boolean finished = false;
    private double initialElevatorPosition = 0;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final Command m_holdingCommand;

    public PostDropCoralL4(Wrist2 wrist, Elevator elevator, double elevatorValue) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        m_holdingCommand = null;
        addRequirements(m_wrist, m_elevator);
    }

    public PostDropCoralL4(Wrist2 wrist, Elevator elevator, double elevatorValue, Command holdingCommand) {
        m_wrist = wrist;
        m_elevator = elevator;
        m_elevatorGoal = elevatorValue;
        m_holdingCommand = holdingCommand;
        addRequirements(m_wrist, m_elevator);
    }

    @Override
    public void initialize() {
        count = 0;
        finished = false;
        // resetting forces the setpoint velocity to zero, which is not always what we
        // want
        // m_wrist.resetWristProfile();
        // m_elevator.resetElevatorProfile();
        
        if(m_holdingCommand != null){
            CommandScheduler.getInstance().cancel(m_holdingCommand);
        }
        initialElevatorPosition = m_elevator.getPosition();
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_elevatorGoal);

        if (Math.abs(m_elevator.getPosition() - initialElevatorPosition) > 2.1) {
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
        if (Experiments.instance.enabled(Experiment.UseProfileDone))
            return finished && m_wrist.profileDone() && m_elevator.profileDone();
        return finished;
    }
}
