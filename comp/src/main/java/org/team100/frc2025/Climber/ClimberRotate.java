
package org.team100.frc2025.Climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberRotate extends Command {
    Climber climber;
    double duty;
    Supplier<Double> m_joystick;

    public ClimberRotate(Climber c, double d, Supplier<Double> joystick) {
        climber = c;
        duty = d;
        m_joystick = joystick;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climber.setDutyCycle(m_joystick.get());
    }

    @Override
    public void end(boolean interrupted) {
        climber.setDutyCycle(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
