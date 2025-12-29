package org.team100.lib.subsystems.five_bar.commands;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidProfileWPI;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.subsystems.five_bar.FiveBarCartesian;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Move in a straight line via interpolation.
 * 
 * This is an example of a command which is too complicated to inline into the
 * subsystem itself, because it has state (the current location, progress
 * towards the endpoint, etc).
 */
public class Move extends Command {

    private final FiveBarCartesian m_fiveBar;
    private final Translation2d m_goal;
    private final IncrementalProfile m_profile;
    private final Timer m_timer;

    private Control100 m_setpoint;
    private Model100 m_profileGoal;

    private Translation2d m_start;
    private double m_distance;
    private boolean m_done;

    public Move(FiveBarCartesian fiveBar, Translation2d goal, double velocity) {
        m_fiveBar = fiveBar;
        m_goal = goal;
        m_profile = new TrapezoidProfileWPI(velocity, 1);

        m_timer = new Timer();
        addRequirements(fiveBar);
    }

    @Override
    public void initialize() {
        m_start = m_fiveBar.getPosition();
        m_distance = m_start.getDistance(m_goal);
        m_setpoint = new Control100();
        m_profileGoal = new Model100(m_distance, 0);
        m_timer.restart();
        m_done = false;
    }

    @Override
    public void execute() {
        m_setpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint, m_profileGoal);
        Control100 c = m_setpoint;
        double s = c.x() / m_distance;
        Translation2d setpoint = m_start.interpolate(m_goal, s);
        double togo = setpoint.getDistance(m_goal);
        if (togo < 0.001) {
            m_fiveBar.setPosition(m_goal);
            m_done = true;
            return;
        }
        m_fiveBar.setPosition(setpoint);
    }

    /**
     * Using our own "done" method instead of the Command "isFinished" method makes
     * it easier to compose.
     */
    public boolean done() {
        return m_done;
    }

}
