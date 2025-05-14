package frc.robot;

import org.team100.lib.motion.lynxmotion_arm.LynxArmKinematics.LynxArmConfig;
import org.team100.lib.profile.timed.JerkLimitedProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Moves the arm in a straight line by interpolating the start and end
 * end-effector poses.
 */
public class MoveCommand extends Command {
    private final LynxArm m_arm;
    private final Pose3d m_goal;
    private final JerkLimitedProfile100 m_profile;
    private final Timer m_timer;

    private Pose3d m_start;
    private double m_distance;
    private boolean m_done;

    public MoveCommand(LynxArm arm, Pose3d goal) {
        m_arm = arm;
        m_goal = goal;
        m_profile = new JerkLimitedProfile100(0.1, 1, 10, true);
        m_timer = new Timer();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_start = m_arm.getPosition().p5();
        // this doesn't work for twist-only moves without the minimum
        m_distance = Math.max(0.01, m_start.getTranslation().getDistance(m_goal.getTranslation()));
        m_profile.init(new Control100(), new Model100(m_distance, 0));
        m_timer.restart();
        // System.out.printf("start %s\n", poseStr(m_start));
        // System.out.printf("end %s\n", poseStr(m_goal));
        m_done = false;
    }

    @Override
    public void execute() {
        // System.out.println("***EXECUTE***");
        Control100 c = m_profile.sample(m_timer.get());
        double s = c.x() / m_distance;
        Pose3d setpoint = m_start.interpolate(m_goal, s);
        // System.out.printf("Setpoint %s\n", poseStr(setpoint));
        double togo = setpoint.getTranslation().getDistance(m_goal.getTranslation());
        Rotation3d rotTogo = setpoint.getRotation().minus(m_goal.getRotation());
        double angleTogo = rotTogo.getAngle();
        Pose3d measurement = m_arm.getPosition().p5();
        Rotation3d rotTogo2 = measurement.getRotation().minus(m_goal.getRotation());
        double angleTogo2 = rotTogo2.getAngle();
        // System.out.printf("to go %f angle %f measured %f \n", togo, angleTogo, angleTogo2);
        if (togo < 0.001 && Math.abs(angleTogo2) < 0.001) {
            // System.out.println("at goal");
            m_arm.setPosition(m_goal);
            m_done = true;
            return;
        }
        m_arm.setPosition(setpoint);
        // System.out.printf("setpoint %s\n", poseStr(setpoint));
        // System.out.printf("goal %s\n", poseStr(m_goal));
        // LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
        // System.out.printf("measured  config %s\n", measuredConfig);
        // LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
        // System.out.printf("commanded config %s\n", commandedConfig);
    }

    public boolean done() {
        return m_done;
    }

    String poseStr(Pose3d p) {
        return String.format("%f %f %f %f %f %f",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }
}
