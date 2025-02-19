package org.team100.lib.config;

/**
 * Be careful of the units for these parameters: different units will be used by
 * different outboard motor controllers, and by the same outboard motor
 * controller when used in different ways. Please add a comment about units to
 * every use of this class.
 */
public class PIDConstants {
    private final double m_positionP;
    private final double m_positionI;
    private final double m_positionD;
    private final double m_positionIZone;
    private final double m_velocityP;
    private final double m_velocityI;
    private final double m_velocityD;
    private final double m_velocityIZone;

    public static PIDConstants makeVelocityPID(double p) {
        return new PIDConstants(0, p);
    }

    public static PIDConstants makePositionPID(double p) {
        return new PIDConstants(p, 0);
    }

    public static PIDConstants makeVelocityPID(double p, double i, double d) {
        return new PIDConstants(0, 0, 0, p, i, d);
    }

    public static PIDConstants makePositionPID(double p, double i, double d) {
        return new PIDConstants(p, i, d, 0, 0, 0);
    }

    public PIDConstants(double positionP, double velocityP) {
        this(positionP, 0, 0, velocityP, 0, 0);
    }

    public PIDConstants() {
        this(0, 0);
    }

    public PIDConstants(double positionP, double positionI, double positionD, double velocityP, double velocityI,
            double velocityD) {
        m_positionP = positionP;
        m_positionI = positionI;
        m_positionD = positionD;
        m_positionIZone = 0;
        m_velocityP = velocityP;
        m_velocityI = velocityI;
        m_velocityD = velocityD;
        m_velocityIZone = 0;
    }

    public double getPositionP() {
        return m_positionP;
    }

    public double getPositionI() {
        return m_positionI;
    }

    public double getPositionD() {
        return m_positionD;
    }

    public double getPositionIZone() {
        return m_positionIZone;
    }

    public double getVelocityP() {
        return m_velocityP;
    }

    public double getVelocityI() {
        return m_velocityI;
    }

    public double getVelocityD() {
        return m_velocityD;
    }

    public double getVelocityIZone() {
        return m_velocityIZone;
    }
}
