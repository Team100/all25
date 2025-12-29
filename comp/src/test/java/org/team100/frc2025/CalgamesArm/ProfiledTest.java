package org.team100.frc2025.CalgamesArm;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.r1.IncrementalProfile;
import org.team100.lib.profile.r1.TrapezoidIncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.subsystems.prr.EAWConfig;
import org.team100.lib.subsystems.prr.ElevatorArmWristKinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ProfiledTest {
    private static final boolean DEBUG = false;
    private static final double DT = 0.02;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    /**
     * This shows that home-to-pick works fine as a profile, because it's almost
     * entirely shoulder motion, nearly circular. Also, since it's in the "reach
     * down" regime of the mechanism, a config profile for this path is much easier
     * to make work.
     * 
     * For charts, see
     * 
     * https://docs.google.com/spreadsheets/d/1yo5gU4NwVDUP8XaGb-7jNOtN6A_7cpX3DWRTfwgKym0/edit?gid=1920529239#gid=1920529239
     */
    @Test
    void homeToPick() {

        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(0.5, 0.343);

        // home position
        EAWConfig start = new EAWConfig(0, 0, 0);
        // floor pick position
        EAWConfig goal = new EAWConfig(0, -3 * Math.PI / 4, Math.PI / 4);

        Model100 g1 = new Model100(goal.shoulderHeight(), 0);
        Model100 g2 = new Model100(goal.shoulderAngle(), 0);
        Model100 g3 = new Model100(goal.wristAngle(), 0);
        IncrementalProfile p1 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);
        IncrementalProfile p2 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);
        IncrementalProfile p3 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);

        Control100 i1 = new Control100(start.shoulderHeight(), 0);
        Control100 i2 = new Control100(start.shoulderAngle(), 0);
        Control100 i3 = new Control100(start.wristAngle(), 0);

        double eta1 = p1.simulateForETA(DT, i1, g1);
        double eta2 = p1.simulateForETA(DT, i2, g2);
        double eta3 = p1.simulateForETA(DT, i3, g3);

        double eta = Math.max(eta1, Math.max(eta2, eta3));

        if (DEBUG)
            System.out.println("t, x, y, r, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < eta; tt += DT) {
            i1 = p1.calculate(DT, i1, g1);
            i2 = p2.calculate(DT, i2, g2);
            i3 = p3.calculate(DT, i3, g3);
            EAWConfig c = new EAWConfig(i1.x(), i2.x(), i3.x());
            Pose2d p = k.forward(c);

            if (DEBUG) {
                System.out.printf(
                        "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        tt, p.getX(), p.getY(), p.getRotation().getRadians(), c.shoulderHeight(),
                        c.shoulderAngle(), c.wristAngle(), i1.v(),
                        i2.v(), i3.v(), i1.a(), i2.a(), i3.a());
            }
        }

    }

    /**
     * This shows why uncoordinated profiles are a bad choice for scoring: the
     * profile goes out too soon, and would end up hitting the scoring posts from
     * the bottom on the way up.
     * 
     * For charts, see
     * 
     * https://docs.google.com/spreadsheets/d/1yo5gU4NwVDUP8XaGb-7jNOtN6A_7cpX3DWRTfwgKym0/edit?gid=1974165792#gid=1974165792
     */
    @Test
    void homeToL4() {

        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(0.5, 0.343);

        // home position
        EAWConfig start = new EAWConfig(0, 0, 0);

        Pose2d pL4 = new Pose2d(1.9, 0.5, new Rotation2d(150));
        // floor pick position
        EAWConfig goal = k.inverse(pL4);

        Model100 g1 = new Model100(goal.shoulderHeight(), 0);
        Model100 g2 = new Model100(goal.shoulderAngle(), 0);
        Model100 g3 = new Model100(goal.wristAngle(), 0);
        IncrementalProfile p1 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);
        IncrementalProfile p2 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);
        IncrementalProfile p3 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);

        Control100 i1 = new Control100(start.shoulderHeight(), 0);
        Control100 i2 = new Control100(start.shoulderAngle(), 0);
        Control100 i3 = new Control100(start.wristAngle(), 0);

        double eta1 = p1.simulateForETA(DT, i1, g1);
        double eta2 = p1.simulateForETA(DT, i2, g2);
        double eta3 = p1.simulateForETA(DT, i3, g3);

        double eta = Math.max(eta1, Math.max(eta2, eta3));

        if (DEBUG)
            System.out.println("t, x, y, r, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < eta; tt += DT) {
            i1 = p1.calculate(DT, i1, g1);
            i2 = p2.calculate(DT, i2, g2);
            i3 = p3.calculate(DT, i3, g3);
            EAWConfig c = new EAWConfig(i1.x(), i2.x(), i3.x());
            Pose2d p = k.forward(c);

            if (DEBUG) {
                System.out.printf(
                        "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        tt, p.getX(), p.getY(), p.getRotation().getRadians(), c.shoulderHeight(),
                        c.shoulderAngle(), c.wristAngle(), i1.v(),
                        i2.v(), i3.v(), i1.a(), i2.a(), i3.a());
            }
        }
    }

    /**
     * Profile down is also not safe.
     */
    @Test
    void l4ToHome() {

        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(0.5, 0.343);
        Pose2d pL4 = new Pose2d(1.9, 0.5, new Rotation2d(150));

        // home position
        EAWConfig start = k.inverse(pL4);

        // floor pick position
        EAWConfig goal = new EAWConfig(0, 0, 0);

        Model100 g1 = new Model100(goal.shoulderHeight(), 0);
        Model100 g2 = new Model100(goal.shoulderAngle(), 0);
        Model100 g3 = new Model100(goal.wristAngle(), 0);
        IncrementalProfile p1 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);
        IncrementalProfile p2 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);
        IncrementalProfile p3 = new TrapezoidIncrementalProfile(log, 1, 1, 0.05);

        Control100 i1 = new Control100(start.shoulderHeight(), 0);
        Control100 i2 = new Control100(start.shoulderAngle(), 0);
        Control100 i3 = new Control100(start.wristAngle(), 0);

        double eta1 = p1.simulateForETA(DT, i1, g1);
        double eta2 = p1.simulateForETA(DT, i2, g2);
        double eta3 = p1.simulateForETA(DT, i3, g3);

        double eta = Math.max(eta1, Math.max(eta2, eta3));

        if (DEBUG)
            System.out.println("t, x, y, r, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < eta; tt += DT) {
            i1 = p1.calculate(DT, i1, g1);
            i2 = p2.calculate(DT, i2, g2);
            i3 = p3.calculate(DT, i3, g3);
            EAWConfig c = new EAWConfig(i1.x(), i2.x(), i3.x());
            Pose2d p = k.forward(c);

            if (DEBUG) {
                System.out.printf(
                        "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        tt, p.getX(), p.getY(), p.getRotation().getRadians(), c.shoulderHeight(), c.shoulderAngle(),
                        c.wristAngle(), i1.v(),
                        i2.v(), i3.v(), i1.a(), i2.a(), i3.a());
            }
        }
    }

}
