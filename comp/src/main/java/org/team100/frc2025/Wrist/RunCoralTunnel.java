package org.team100.frc2025.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class RunCoralTunnel extends Command {
    double m_value;
    CoralTunnel m_tunnel;

    public RunCoralTunnel(CoralTunnel tunnel, double value) {
        m_tunnel = tunnel;
        m_value = value;
        addRequirements(m_tunnel);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // m_wrist.setAlgaeMotor(0.5);
        m_tunnel.setCoralMotor(m_value);

        // System.out.println("Coral Tunnel Motor MOOOVING");

    }

    @Override
    public void end(boolean interrupted) {
        // m_wrist.setCoralMotor(0);
        m_tunnel.setCoralMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
