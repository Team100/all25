package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeAlgaeGrip extends Command {
    AlgaeGrip m_grip;
    Timer m_timer = new Timer();
    boolean atit = false;
    public OuttakeAlgaeGrip(AlgaeGrip grip) {
        m_grip = grip;
        addRequirements(m_grip);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        atit = false;
        m_grip.applyHighConfigs();
        // m_grip.setDutyCycle(-1);
    }

    @Override
    public void execute() {
        if(m_timer.get() < 0.5){
            m_grip.setDutyCycle(1);
        }else{
            m_grip.applyLowConfigs();
            atit = true;
        }

        if(atit){
            m_grip.setDutyCycle(-1);
        }


        
    }

    public void end(boolean interrupted) {
        m_grip.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
