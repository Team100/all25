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

public class GoToAndRunFunnel extends ParallelCommandGroup {
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
        addCommands(
                new ProfileToCoralStation(logger, drive, heedRadiusM, controller, profile, velocitySupplier),
                new RunFunnelHandoff(logger, elevator, wrist, funnel, tunnel, grip));
    }
}
