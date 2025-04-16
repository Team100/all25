package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;

public class MockBareMotor implements BareMotor {
    public double output = 0;
    public double velocity = 0;
    public double position = 0;
    public double accel = 0;
    public double torque = 0;

    /**
     * this is for testing feedforwards.
     */
    public double ffVolts;
    public double frictionFFVolts;
    public double velocityFFVolts;
    public double torqueFFVolts;
    public double accelFFVolts;
    private final Feedforward100 m_ff;

    public MockBareMotor(Feedforward100 ff) {
        m_ff = ff;
    }

    @Override
    public void setDutyCycle(double output) {
        this.output = output;
    }

    @Override
    public void setVelocity(double velocityRad_S, double accel, double torque) {
        this.velocity = velocityRad_S;
        this.accel = accel;
        this.torque = torque;
    }

    @Override
    public void setPosition(double position, double velocity, double accelRad_S2, double torque) {
        this.position = position;
        this.velocity = velocity;
        this.accel = accelRad_S2;
        this.torque = torque;

        final double motorRev_S = velocity / (2 * Math.PI);
        final double motorRev_S2 = accelRad_S2 / (2 * Math.PI);

        // use the setpoint as the measurement
        final double currentMotorRev_S = velocity;

        frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        torqueFFVolts = getTorqueFFVolts(torque);
        accelFFVolts = m_ff.accelFFVolts(currentMotorRev_S, motorRev_S2);
        ffVolts = frictionFFVolts + velocityFFVolts + torqueFFVolts + accelFFVolts;

    }

    /** placeholder */
    @Override
    public double kROhms() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kTNm_amp() {
        return 0.02;
    }

    @Override
    public void stop() {
        this.output = 0;
        this.velocity = 0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getVelocityRad_S() {
        return this.velocity;
    }

    @Override
    public void setEncoderPositionRad(double positionRad) {
        this.position = positionRad;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        //
    }

    @Override
    public void periodic() {
        //
    }

}
