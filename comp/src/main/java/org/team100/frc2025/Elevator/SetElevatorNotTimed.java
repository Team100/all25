package org.team100.frc2025.Elevator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorNotTimed extends Command implements Glassy {
    private final Elevator m_elevator;
    private final double m_value;
    private final boolean m_perpetual;
    private final DoubleLogger m_log_error;

    public SetElevatorNotTimed(LoggerFactory logger, Elevator elevator, double value, boolean perpetual) {
        LoggerFactory child = logger.child(this);
        m_log_error = child.doubleLogger(Level.TRACE, "error");
        m_elevator = elevator;
        m_value = value;
        m_perpetual = perpetual;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.resetElevatorProfile();
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_value); // 24.5 for l3
        double error = Math.abs(m_elevator.getPosition() - m_value);
        m_log_error.log(() -> error);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
    }

    @Override
    public boolean isFinished() {
        if (m_perpetual) {
            return false;
        } else {
            return m_elevator.atGoal();
        }
    }
}
