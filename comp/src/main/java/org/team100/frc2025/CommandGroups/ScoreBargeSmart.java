package org.team100.frc2025.CommandGroups;

import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.Wrist2;

import edu.wpi.first.wpilibj2.command.Command;

public class ScoreBargeSmart extends Command {
    Elevator m_elevator;
    Wrist2 m_wrist;
    Supplier<Boolean> m_readyToShoot;
    AlgaeGrip m_grip;

    public ScoreBargeSmart(Elevator elevator, Wrist2 wrist, AlgaeGrip grip, Supplier<Boolean> readyToShoot) {
        m_elevator = elevator;
        m_wrist = wrist;
        m_readyToShoot = readyToShoot;
        m_grip = grip;
        addRequirements(m_elevator, m_wrist, m_grip);
    }

    @Override
    public void initialize() {
        m_grip.applyHighConfigs();
    }

    @Override
    public void execute() {
        if (!m_readyToShoot.get()) {
            m_wrist.setAngleValue(2.0);
            m_elevator.setPosition(20.0);

            m_grip.setDutyCycle(0.5);

        if(m_grip.hasAlgae()){
            m_grip.applyLowConfigs();
        }
    } else {
      m_elevator.setPosition(54.0);
      m_wrist.setAngleValue(2.2);

            if (Math.abs(m_elevator.getPosition() - 54) < 0.5) {
                m_grip.applyHighConfigs();
                m_grip.setDutyCycle(-1);
            }
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
