package org.team100.frc2025.Elevator;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDefaultCommand extends Command {
    private final Elevator m_elevator;
    private final Wrist2 m_wrist;
    private final AlgaeGrip m_grip;
    private final SwerveDriveSubsystem m_drive;

    public ElevatorDefaultCommand(Elevator elevator, Wrist2 wrist, AlgaeGrip grip, SwerveDriveSubsystem drive) {
        m_elevator = elevator;
        m_wrist = wrist;
        m_grip = grip;
        m_drive = drive;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.resetElevatorProfile();

    }

    @Override
    public void execute() {

        double distanceToReef = FieldConstants.getDistanceToReefCenter(m_drive.getPose().getTranslation());

        if (distanceToReef > 1.6) {

            if (!m_grip.hasAlgae()) {
                double goal = 0.05;

                if (m_wrist.getSafeCondition()) {
                    m_elevator.setPositionNoGravity(goal);
                } else {
                    m_elevator.setStatic();
                }

                double error = Math.abs(m_elevator.getPosition() - goal);

                if (error <= 0.3) {
                    m_elevator.setSafeCondition(true);
                    // m_elevator.setPosition(goal);

                } else {
                    m_elevator.setSafeCondition(false);
                    // m_elevator.setPosition(goal);

                }
            } else {
                double goal = 12;

                if (m_wrist.getSafeCondition()) {
                    m_elevator.setPosition(goal);
                } else {
                    m_elevator.setStatic();
                }

                // double error = Math.abs(m_elevator.getPosition() - goal);

                // if(error <= 0.2){
                // m_elevator.setSafeCondition(true);
                // } else{
                // m_elevator.setSafeCondition(false);
                // }

            }
        } else {
            m_elevator.setStatic();
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
