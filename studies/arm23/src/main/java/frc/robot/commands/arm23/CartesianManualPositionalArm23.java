package org.team100.lib.commands.arm23;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ArmAnglesLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Translation2dLogger;
import org.team100.lib.motion.arm23.ArmAngles23;
import org.team100.lib.motion.arm23.ArmKinematics23;
import org.team100.lib.motion.arm23.ArmSubsystem23;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive the arm to the specified cartesian position.
 * 
 * There are several flaws in this implementation.
 */
public class CartesianManualPositionalArm23 extends Command implements Glassy {

    private final ArmSubsystem23 m_arm;
    private final ArmKinematics23 m_kinematics;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;

    private final PIDFeedback m_lowerFeedback;
    private final PIDFeedback m_upperFeedback;

    // LOGGERS

    private final Translation2dLogger m_log_input;
    private final ArmAnglesLogger m_log_setpoint;
    private final ArmAnglesLogger m_log_measurement;
    private final Translation2dLogger m_log_cartesian_measurement;
    private final DoubleLogger m_log_output_u1;
    private final DoubleLogger m_log_output_u2;

    public CartesianManualPositionalArm23(
            LoggerFactory parent,
            ArmSubsystem23 arm,
            ArmKinematics23 kinematics,
            DoubleSupplier x,
            DoubleSupplier y) {
        LoggerFactory child = parent.child(this);
        m_log_input = child.translation2dLogger(Level.TRACE, "input");
        m_log_setpoint = child.armAnglesLogger(Level.TRACE, "setpoint");
        m_log_measurement = child.armAnglesLogger(Level.TRACE, "measurement");
        m_log_cartesian_measurement = child.translation2dLogger(Level.TRACE, "cartesian_measurement");
        m_log_output_u1 = child.doubleLogger(Level.TRACE, "output/u1");
        m_log_output_u2 = child.doubleLogger(Level.TRACE, "output/u2");

        m_arm = arm;
        m_kinematics = kinematics;
        m_x = x;
        m_y = y;

        m_lowerFeedback = new PIDFeedback(child.child("lower"), 1, 0, 0, true, 0.05, 1);
        m_upperFeedback = new PIDFeedback(child.child("upper"), 1, 0, 0, true, 0.05, 1);
        addRequirements(arm);
    }

    /**
     * Use inverse kinematics to transform the manual input into joint space.
     * This uses an offset to keep the inputs away from the origin.
     */
    @Override
    public void execute() {
        final Translation2d input = new Translation2d(
                0.6 * m_x.getAsDouble() + 0.7,
                0.6 * m_y.getAsDouble() + 0.7);

        final ArmAngles23 setpoint = m_kinematics.inverse(input);
        if (setpoint == null) {
            Util.warn("Ignoring infeasible input");
            return;
        }

        Optional<ArmAngles23> measurement = m_arm.getPosition();
        if (measurement.isEmpty())
            return;

        final Translation2d cartesian_measurement = m_kinematics.forward(measurement.get());

        final double u1 = MathUtil.clamp(
                m_lowerFeedback.calculate(
                        new Model100(measurement.get().th1(), 0),
                        new Model100(setpoint.th1(), 0)),
                -1, 1);
        final double u2 = MathUtil.clamp(
                m_upperFeedback.calculate(
                        new Model100(measurement.get().th2(), 0),
                        new Model100(setpoint.th2(), 0)),
                -1, 1);

        m_arm.set(u1, u2);

        m_log_input.log(() -> input);
        m_log_setpoint.log(() -> setpoint);
        m_log_measurement.log(measurement::get);
        m_log_cartesian_measurement.log(() -> cartesian_measurement);
        m_log_output_u1.log(() -> u1);
        m_log_output_u2.log(() -> u2);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.set(0, 0);
    }

}
