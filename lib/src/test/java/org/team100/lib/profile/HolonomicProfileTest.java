package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveControl;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class HolonomicProfileTest {
    private static final boolean PRINT = false;

    @Test
    void test2d() {
        HolonomicProfile hp = new HolonomicProfile(1, 1, 0.01, 1, 1, 0.01);
        SwerveModel i = new SwerveModel();
        SwerveModel g = new SwerveModel(new Pose2d(1, 5, Rotation2d.kZero));
        hp.solve(i, g);
        SwerveControl s = i.control();
        for (double t = 0; t < 10; t += 0.02) {
            s = hp.calculate(s.model(), g);
            if (PRINT)
                Util.printf("%.2f %.3f %.3f\n", t, s.x().x(), s.y().x());
        }
    }

    @Test
    void test2dWithEntrySpeed() {
        HolonomicProfile hp = new HolonomicProfile(1, 1, 0.01, 1, 1, 0.01);
        SwerveModel i = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(1, 0, 0));
        SwerveModel g = new SwerveModel(new Pose2d(0, 1, Rotation2d.kZero));
        hp.solve(i, g);
        SwerveControl s = i.control();
        for (double t = 0; t < 10; t += 0.02) {
            s = hp.calculate(s.model(), g);
            if (PRINT)
                Util.printf("%.2f %.3f %.3f\n", t, s.x().x(), s.y().x());
        }
    }

    /**
     * On my desktop, the solve() method takes about 1 microsecond, so it seems
     * ok to not worry about how long it takes.
     */
    @Test
    void testSolvePerformance() {
        HolonomicProfile hp = new HolonomicProfile(1, 1, 0.01, 1, 1, 0.01);
        SwerveModel i = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(1, 0, 0));
        SwerveModel g = new SwerveModel(new Pose2d(0, 1, Rotation2d.kZero));
        int N = 1000000;
        double t0 = Takt.actual();
        for (int ii = 0; ii < N; ++ii) {
            hp.solve(i, g);
        }
        double t1 = Takt.actual();
        if (PRINT)
            Util.printf("duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
        if (PRINT)
            Util.printf("per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);
    }
}
