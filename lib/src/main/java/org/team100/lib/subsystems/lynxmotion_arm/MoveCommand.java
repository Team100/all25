package org.team100.lib.subsystems.lynxmotion_arm;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidProfileWPI;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Moves the arm in a straight line by interpolating the start and end
 * end-effector poses.
 */
public class MoveCommand extends Command {
    private static final boolean DEBUG = false;
    private final LynxArm m_arm;
    private final Pose3d m_goal;
    private final IncrementalProfile m_profile;
    private final Timer m_timer;

    private Control100 m_setpoint;
    private Model100 m_profileGoal;

    private Pose3d m_start;
    private double m_grip;
    private double m_distance;
    private boolean m_done;

    public MoveCommand(LynxArm arm, Pose3d goal, double velocity) {
        m_arm = arm;
        m_goal = goal;
        m_profile = new TrapezoidProfileWPI(velocity, 1);
        m_timer = new Timer();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (DEBUG)
            System.out.println("\n***INITIALIZE***");
        m_start = m_arm.getPosition().p6();
        m_grip = m_arm.getGrip();

        // this doesn't work for twist-only moves without the minimum
        m_distance = Math.max(0.01, m_start.getTranslation().getDistance(m_goal.getTranslation()));
        m_setpoint = new Control100();
        m_profileGoal = new Model100(m_distance, 0);
        m_timer.restart();
        m_done = false;
        if (DEBUG) {
            System.out.printf("start %s\n", StrUtil.poseStr(m_start));
            System.out.printf("end %s\n", StrUtil.poseStr(m_goal));
        }
    }

    @Override
    public void execute() {
        if (DEBUG)
            System.out.println("\n***EXECUTE***");
        // the servo doesn't need to be commanded to maintain its position but it's a
        // good habit since some motor types do need this.
        m_arm.setGrip(m_grip);
        m_setpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint, m_profileGoal);
        Control100 c = m_setpoint;
        double s = c.x() / m_distance;
        Pose3d setpoint = GeometryUtil.interpolate(m_start, m_goal, s);
        Pose3d measurement = m_arm.getPosition().p6();
        if (DEBUG) {
            System.out.printf("Goal %s Setpoint %s Measurement %s\n",
                    StrUtil.poseStr(m_goal), StrUtil.poseStr(setpoint), StrUtil.poseStr(measurement));
        }
        double togo = setpoint.getTranslation().getDistance(m_goal.getTranslation());
        Rotation3d rotTogo = setpoint.getRotation().minus(m_goal.getRotation());
        double angleTogo = rotTogo.getAngle();
        Rotation3d rotTogo2 = measurement.getRotation().minus(m_goal.getRotation());
        double angleTogo2 = rotTogo2.getAngle();
        if (DEBUG)
            System.out.printf("to go %f angle %f measured %f \n", togo, angleTogo,
                    angleTogo2);
        // "done" state ignores the measurement to avoid solver convergence.
        if (togo < 0.001 && Math.abs(angleTogo) < 0.001) {
            if (DEBUG)
                System.out.println("***** at goal *****");
            m_arm.setPosition(m_goal);
            m_done = true;
            return;
        }
        m_arm.setPosition(setpoint);
        if (DEBUG) {
            System.out.printf("setpoint %s\n", StrUtil.poseStr(setpoint));
            System.out.printf("goal %s\n", StrUtil.poseStr(m_goal));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            System.out.printf("measured config %s\n", measuredConfig.str());
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            System.out.printf("commanded config %s\n", commandedConfig.str());
        }
    }

    public boolean done() {
        return m_done;
    }

}
