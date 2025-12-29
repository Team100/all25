package org.team100.lib.subsystems.lynxmotion_arm;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidProfileWPI;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveCommandTwoDof extends Command {
    private final LynxArmTwoDof m_arm;
    private final Translation2d m_goal;
    private final IncrementalProfile m_profile;
    private final Timer m_timer;

    private Control100 m_setpoint;
    private Model100 m_profileGoal;

    private Translation2d m_start;
    private double m_distance;
    private boolean m_done;

    public MoveCommandTwoDof(LynxArmTwoDof arm, Translation2d goal) {
        m_arm = arm;
        m_goal = goal;
        m_profile = new TrapezoidProfileWPI(0.1, 1);
        m_timer = new Timer();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_start = m_arm.getPosition().p2();
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
            m_arm.setPosition(m_goal);
            m_done = true;
            return;
        }
        m_arm.setPosition(setpoint);
    }

    public boolean done() {
        return m_done;
    }

}
