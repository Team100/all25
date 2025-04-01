// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.CommandGroups;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.Auto.ProfileToCoralStation;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToAndRunFunnel extends ParallelCommandGroup {
  /** Creates a new GoToAndRunFunnel. */
  public GoToAndRunFunnel(LoggerFactory logger,
            Elevator elevator,
            Wrist2 wrist,
            Funnel funnel,
            CoralTunnel tunnel,
            AlgaeGrip grip,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            SwerveController controller,
            HolonomicProfile profile,
            Supplier<DriverControl.Velocity> velocitySupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ProfileToCoralStation(logger, drive, heedRadiusM, controller, profile, velocitySupplier),
        new RunFunnelHandoff(logger, elevator, wrist, funnel, tunnel, grip)
    );
  }
}
