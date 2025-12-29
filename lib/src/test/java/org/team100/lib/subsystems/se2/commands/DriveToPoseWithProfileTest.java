package org.team100.lib.subsystems.se2.commands;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.FullStateControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.se2.HolonomicProfile;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.MockSubsystemSE2;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Demonstrate DriveToPoseWithProfile.
 * 
 * https://docs.google.com/spreadsheets/d/1tt7Fq-gkR7aoY6kH2WFVxj4y__SMXiKE0scPG3eHSAk/edit?gid=2109422227#gid=2109422227
 */
public class DriveToPoseWithProfileTest implements Timeless {
    private static final boolean DEBUG = false;
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    MockSubsystemSE2 subsystem = new MockSubsystemSE2(new ModelSE2());
    FullStateControllerSE2 controller = ControllerFactorySE2.test(log);
    HolonomicProfile profile = HolonomicProfileFactory.wpi(1, 1, 1, 1);

    /**
     * the goal can be a constant, as shown here, or it can be updated somehow (e.g.
     * by camera or button input), which is why it is a Supplier.
     */
    @Test
    void testDemo() {
        Command drive = new DriveToPoseWithProfile(log, subsystem, controller, profile,
                () -> new Pose2d(1, 2, new Rotation2d(Math.PI / 2)));
        drive.initialize();
        stepTime();
        System.out.println("x, y, theta");
        for (int i = 0; i < 200; ++i) {
            drive.execute();
            subsystem.m_state = new ModelSE2(subsystem.m_state.pose(), subsystem.m_setpoint);
            subsystem.m_state = subsystem.m_state.evolve(0.02);
            Pose2d p = subsystem.m_state.pose();
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f\n", p.getX(), p.getY(), p.getRotation().getRadians());
            stepTime();
        }
    }

}
