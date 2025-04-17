package org.team100.lib.examples.semiauto;

import java.util.function.Supplier;

import org.team100.lib.hid.DriverControl;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is an example of what you'd put in RobotContainer to use some of the
 * subsystem and command classes here.
 */
public class ContainerSnippet {
    public ContainerSnippet() {
        DriverControl control = new DriverControl() {
        };
        // The actual pose provider would be the swerve subsystem
        Supplier<Pose2d> pose = () -> new Pose2d();
        PositionAwareSubsytem shooterElevation = new PositionAwareSubsytem(
                new Translation2d(3, 4), pose);

        // you could run the elevation command as the default ...
        shooterElevation.setDefaultCommand(shooterElevation.holdAimingPoint());

        // ... or you could run it on demand:
        new Trigger(control::button4).whileTrue(shooterElevation.holdAimingPoint());
    }

}
