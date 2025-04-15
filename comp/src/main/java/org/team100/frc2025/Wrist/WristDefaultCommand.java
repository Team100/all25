package org.team100.frc2025.Wrist;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StringLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.TrackingTimedProfileReference1d;
import org.team100.lib.state.Model100;

import edu.wpi.first.wpilibj2.command.Command;

public class WristDefaultCommand extends Command implements Glassy {
    private final double kPositionTolerance = 0.02;
    private final double kVelocityTolerance = 0.01;

    private final StringLogger m_log_activity;
    private final Elevator m_elevator;
    private final Wrist2 m_wrist;
    private static final double deadband = 0.03;
    private final AlgaeGrip m_grip;
    private final SwerveDriveSubsystem m_drive;

    private double count = 0;
    private boolean docked = false;
    private double m_holdPosition;
    private final DoubleLogger m_log_holdPosition;
    private TrackingTimedProfileReference1d m_ref;
    private Model100 m_setpoint;
    private Model100 m_goal;

    public WristDefaultCommand(LoggerFactory logger, Wrist2 wrist, Elevator elevator, AlgaeGrip grip,
            SwerveDriveSubsystem drive) {
        LoggerFactory child = logger.child(this);
        m_log_activity = child.stringLogger(Level.TRACE, "activity");
        m_log_holdPosition = child.doubleLogger(Level.TRACE, "hold position (m)");

        m_elevator = elevator;
        m_wrist = wrist;
        m_grip = grip;
        m_drive = drive;
        m_ref = wrist.updatableReference();
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        // we *only* reset the profile in the default command.
        // this sets the setpoint position to the current measurement,
        // and sets the setpoint velocity to zero.
        m_wrist.resetWristProfile();
        count = 0;
        docked = false;
        m_holdPosition = m_wrist.getAngle();
        Model100 initial = new Model100(m_wrist.getAngle(), 0);
        m_ref.init(initial, initial);
    }

    private void goTo(double val) {
        Model100 goal = new Model100(val, 0);
        if (goal.near(m_goal, kPositionTolerance, kVelocityTolerance)) {
            // don't reinitialize ref
        } else {
            m_ref.init(m_setpoint, goal);
            m_goal = goal;
        }
        m_wrist.setAngleSetpoint(m_ref.get());
    }

    @Override
    public void execute() {

        m_log_holdPosition.log(() -> m_holdPosition);
        double distanceToReef = FieldConstants.getDistanceToReefCenter(m_drive.getPose().getTranslation());
        if (distanceToReef < 1.6) {
            // System.out.println("IM TOO CLOSE HOLD");
            m_wrist.setAngleSetpoint(m_ref.get());
            // m_wrist.setAngleValue(m_holdPosition);
            m_log_activity.log(() -> "far away");
            return;
        }

        m_holdPosition = m_wrist.getAngle();
        Model100 initial = new Model100(m_wrist.getAngle(), 0);
        m_ref.init(initial, initial);

        // if (distanceToReef > 1.6) {
        if (!m_grip.hasAlgae()) {
            m_log_activity.log(() -> "no algae");
            if (m_elevator.getPosition() > 17.5) {
                // System.out.println("IM OVER 17.5");

                // m_wrist.setAngleValue(0.5);
                goTo(0.5);

                if (m_wrist.getAngle() < 0.5 - deadband) {
                    m_wrist.setSafeCondition(false);
                    // m_wrist.setAngleValue(0.5);
                } else if (m_wrist.getAngle() > 0.5 - deadband && m_wrist.getAngle() < 1.78 + deadband) {
                    m_wrist.setSafeCondition(true);
                    // m_wrist.setAngleValue(0.5);
                } else if (m_wrist.getAngle() > 1.78) {
                    m_wrist.setSafeCondition(false);
                    // m_wrist.setAngleValue(0.5);
                }
            } else if (m_elevator.getPosition() > 2 && m_elevator.getPosition() < 17.5) {
                // System.out.println("IM WITHIN THE FIRST STAGE");

                // m_wrist.setAngleValue(0.5);
                goTo(0.5);

                if (m_wrist.getAngle() < 0.5 - deadband) {
                    m_wrist.setSafeCondition(false);
                    // m_wrist.setAngleValue(0.5);
                } else if (m_wrist.getAngle() > 0.5 - deadband && m_wrist.getAngle() < 1.78 + deadband) {
                    m_wrist.setSafeCondition(true);
                    // m_wrist.setAngleValue(0.5);
                } else if (m_wrist.getAngle() > 1.78) {
                    m_wrist.setSafeCondition(false);
                    // m_wrist.setAngleValue(0.5);
                }
            } else {
                // System.out.println("IM GOING IN");
                if (m_elevator.getSafeCondition()) {
                    m_wrist.setSafeCondition(true);
                    // m_wrist.setAngleValue(0.1);
                    goTo(0.1);
                }
            }
        } else {
            m_log_activity.log(() -> "has algae");
            // m_wrist.setAngleValue(3.7);
            goTo(3.7);

            double error = Math.abs(m_wrist.getAngle() - 3.7);

            if (error < 0.1) {
                m_wrist.setSafeCondition(true);
            }
        }
        // } else {
        // m_log_activity.log(() -> "far away");
        // // m_wrist.stop();
        // m_wrist.setAngleValue(0.5);
        // }

        // m_wrist.setAngleValue(0.4);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
