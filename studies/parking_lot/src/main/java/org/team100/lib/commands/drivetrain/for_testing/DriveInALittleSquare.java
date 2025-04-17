package org.team100.lib.commands.drivetrain.for_testing;

import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Makes a little square, one meter on a side, forever.
 * 
 * This is intended for tuning the steering control stack, because at each
 * corner of the square, the steering needs to respond quickly and precisely.
 * 
 * It samples a jerk-limited profile for driving, with feedforward to the
 * velocity servo but no positional feedback.
 * 
 * It sends the steering position servo fixed goals, so the servo profile is
 * used.
 */
public class DriveInALittleSquare extends Command implements Glassy  {
    enum DriveState {
        DRIVING,
        STEERING
    }

    /** square should be 1 m on a side. */
    private static final double kDriveLengthM = 1;
    private static final double kMaxVel = 1;
    private static final double kMaxAccel = 1;
    private static final double kXToleranceRad = 0.02;
    private static final Control100 kStart = new Control100(0, 0);
    private static final Model100 kGoal = new Model100(kDriveLengthM, 0);
    private static final double kVToleranceRad_S = 0.02;

    private final SwerveDriveSubsystem m_drive;
    private final Profile100 m_driveProfile;

    /** Current speed setpoint. */
    Control100 m_setpoint;
    /** Current swerve steering axis goal. */
    Rotation2d m_goal;
    DriveState m_state;

    public DriveInALittleSquare(SwerveDriveSubsystem swerve) {
        m_drive = swerve;
        m_driveProfile = new TrapezoidProfile100(kMaxVel, kMaxAccel, 0.05);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // First get the wheels pointing the right way.
        m_state = DriveState.STEERING;
        m_goal = Rotation2d.kZero;
        m_setpoint = kStart;
        m_setpoint = m_driveProfile.calculate(0, m_setpoint.model(), kGoal);
    }

    @Override
    public void execute() {
        switch (m_state) {
            case DRIVING:
                if (MathUtil.isNear(m_setpoint.x(), kGoal.x(), kXToleranceRad)
                        && MathUtil.isNear(m_setpoint.v(), kGoal.v(), kVToleranceRad_S)) {
                    // we were driving, but the timer elapsed, so switch to steering
                    m_state = DriveState.STEERING;
                    m_goal = m_goal.plus(Rotation2d.kCCW_Pi_2);
                    m_setpoint = kStart;
                } else {
                    // keep going
                    m_setpoint = m_driveProfile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint.model(), kGoal);
                }
                break;
            case STEERING:
                if (Util.all(m_drive.getSwerveLocal().atGoal())) {
                    // we were steering, but all the setpoints have been reached, so switch to
                    // driving
                    m_state = DriveState.DRIVING;
                    m_setpoint = kStart;
                    m_setpoint = m_driveProfile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint.model(), kGoal);
                } else {
                    // wait to reach the setpoint
                }
                break;
        }

        // there are four states here because state is mutable :-(
        SwerveModuleStates states = new SwerveModuleStates(
                new SwerveModuleState100(m_setpoint.v(), Optional.of(m_goal)),
                new SwerveModuleState100(m_setpoint.v(), Optional.of(m_goal)),
                new SwerveModuleState100(m_setpoint.v(), Optional.of(m_goal)),
                new SwerveModuleState100(m_setpoint.v(), Optional.of(m_goal))
        );
        m_drive.setRawModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
