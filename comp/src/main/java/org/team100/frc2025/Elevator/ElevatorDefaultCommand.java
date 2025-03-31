package org.team100.frc2025.Elevator;

import org.team100.frc2025.FieldConstants;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDefaultCommand extends Command implements Glassy {
    private final DoubleLogger m_log_distanceToReef;
    private final Elevator m_elevator;
    private final Wrist2 m_wrist;
    private final AlgaeGrip m_grip;
    private final SwerveDriveSubsystem m_drive;
    private double m_holdPosition;
    private final DoubleLogger m_log_holdPosition;

    public ElevatorDefaultCommand(LoggerFactory logger, Elevator elevator, Wrist2 wrist, AlgaeGrip grip,
            SwerveDriveSubsystem drive) {
        LoggerFactory child = logger.child(this);
        m_log_distanceToReef = child.doubleLogger(Level.TRACE, "distance to reef (m)");
        m_log_holdPosition = child.doubleLogger(Level.TRACE, "hold position (m)");

        m_elevator = elevator;
        m_wrist = wrist;
        m_grip = grip;
        m_drive = drive;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        // only the default command resets the profile
        m_elevator.resetElevatorProfile();
        m_holdPosition = m_elevator.getPosition();
    }

    @Override
    public void execute() {
        double distanceToReef = FieldConstants.getDistanceToReefCenter(m_drive.getPose().getTranslation());
        m_log_distanceToReef.log(() -> distanceToReef);

        if(!m_wrist.getSafeCondition()){
            // elevator shouldn't move at all
            m_elevator.setPositionDirectly(m_holdPosition);

            double goal = 0;
            if(!m_grip.hasAlgae()){
                goal = 0.2;
            } else {
                goal = 12;
            }

            double error = Math.abs(m_elevator.getPosition() - goal);
            if(error <= 0.3){
                m_elevator.setSafeCondition(true);
            } else {
                m_elevator.setSafeCondition(false);
            }
            
            return;
        }

        if(distanceToReef < 1.6){
            m_elevator.setPositionDirectly(m_holdPosition);
            return;
        }


        m_holdPosition = m_elevator.getPosition();

        // if (distanceToReef > 1.6) {

            if (!m_grip.hasAlgae()) {
                double goal = 0.2;

                if (m_wrist.getSafeCondition()) {
                    m_elevator.setPositionNoGravity(goal);
                } else {
                    m_elevator.setStatic();
                }

                double error = Math.abs(m_elevator.getPosition() - goal);

                if (error <= 0.3) {
                    m_elevator.setSafeCondition(true);

                } else {
                    m_elevator.setSafeCondition(false);

                }
            } else {
                double goal = 12;

                if (m_wrist.getSafeCondition()) {
                    m_elevator.setPosition(goal);
                } else {
                    m_elevator.setStatic();
                }

            }
        // } else {
        //     m_elevator.setPositionDirectly(m_holdPosition);
        // }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
