package org.team100.frc2025.Elevator;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;

import edu.wpi.first.wpilibj2.command.Command;

public class SetElevator extends Command implements Glassy {
    private static final double maxVel = 220; // 220
    private static final double maxAccel = 200; // 240
    private static final double kPositionTolerance = 0.02;
    private static final double kVelocityTolerance = 0.02;

    private final Elevator m_elevator;
    private final double m_value;
    private final boolean m_perpetual;
    private final IntLogger m_log_count;
    private final DoubleLogger m_log_error;

    private boolean finished = false;
    private int count = 0;

    public SetElevator(LoggerFactory logger, Elevator elevator, double value, boolean perpetual) {
        LoggerFactory child = logger.child(this);
        m_log_count = child.intLogger(Level.TRACE, "count");
        m_log_error = child.doubleLogger(Level.TRACE, "error");
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
        // resetting forces the setpoint velocity to zero, which is not always what we
        // want
        // m_elevator.resetElevatorProfile();
    }

    @Override
    public void execute() {
        m_elevator.setPosition(m_value); // 24.5 for l3

        double error = Math.abs(m_elevator.getPosition() - m_value);
        m_log_error.log(() -> error);
        if (error < 0.5) {
            count++;
        } else {
            count = 0;
        }

        m_log_count.log(() -> count);
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
        }
        if (Experiments.instance.enabled(Experiment.UseProfileDone))
            return finished && m_elevator.profileDone();
        return finished;

    }
}
