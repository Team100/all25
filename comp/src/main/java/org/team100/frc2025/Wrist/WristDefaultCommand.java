package org.team100.frc2025.Wrist;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class WristDefaultCommand extends Command {
    Elevator m_elevator;
    Wrist2 m_wrist;
    double deadband = 0.03;
    double count = 0;
    boolean docked = false;
    AlgaeGrip m_grip;
    SwerveDriveSubsystem m_drive;

    public WristDefaultCommand(Wrist2 wrist, Elevator elevator, AlgaeGrip grip, SwerveDriveSubsystem drive) {
        m_elevator = elevator;
        m_wrist = wrist;
        m_grip = grip;
        m_drive = drive;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_wrist.resetWristProfile();
        count = 0;
        docked = false;

    }

    @Override
    public void execute() {
        double distanceToReef = FieldConstants.getDistanceToReefCenter(m_drive.getPose().getTranslation());
        if (distanceToReef > 1.6) {
            if (!m_grip.hasAlgae()) {
                if (m_elevator.getPosition() > 17.5) {
                    m_wrist.setAngleValue(0.5);
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
                    m_wrist.setAngleValue(0.5);
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
                    if (m_elevator.getSafeCondition()) {
                        m_wrist.setSafeCondition(true);
                        m_wrist.setAngleValue(0.1);
                    }
                }
            } else {
                m_wrist.setAngleValue(3.7);

                double error = Math.abs(m_wrist.getAngle() - 3.7);

                if (error < 0.1) {
                    m_wrist.setSafeCondition(true);
                }
            }
        } else {
            m_wrist.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
