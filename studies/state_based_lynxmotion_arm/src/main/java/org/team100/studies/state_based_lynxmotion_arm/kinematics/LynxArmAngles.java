package org.team100.studies.state_based_lynxmotion_arm.kinematics;

/** Represents the state of a Lynx arm. */
public class LynxArmAngles {
    public static class Config {
        public double swingCenter = 0.5;
        public double swingScale = Math.PI;
        public double boomCenter = 0.5;
        public double boomScale = 4;
        public double stickOffset = 0.0;
        public double stickScale = Math.PI;
        /** Zero rad is this in servo level */
        // double wristCenter = 0.55;
        public double wristCenter = 0.6;
        /** from 0 to 1 is what angle */
        public double wristScale = Math.PI;
    }

    public static class Factory {
        private final Config m_config;

        public Factory() {
            this(new Config());
        }

        public Factory(Config config) {
            m_config = config;
        }

        /**
         * @param swingRad
         * @param boomRad
         * @param stickRad
         * @param wristRad
         * @param twist0_1 passthrough not rad
         * @param grip0_1  passthrough not rad
         */
        public LynxArmAngles fromRad(
                double swingRad,
                double boomRad,
                double stickRad,
                double wristRad,
                double twist0_1,
                double grip0_1) {
            return new LynxArmAngles(
                    m_config,
                    fromSwingRad(swingRad),
                    fromBoomRad(boomRad),
                    fromStickRad(stickRad),
                    fromWristRad(wristRad),
                    twist0_1,
                    grip0_1);
        }

        public LynxArmAngles fromDeg(
                double swingDeg,
                double boomDeg,
                double stickDeg,
                double wristDeg,
                double twist0_1,
                double grip0_1) {
            return fromRad(
                    Math.toRadians(swingDeg),
                    Math.toRadians(boomDeg),
                    Math.toRadians(stickDeg),
                    Math.toRadians(wristDeg),
                    twist0_1,
                    grip0_1);
        }

        public LynxArmAngles from0_1(
                double swing0_1,
                double boom0_1,
                double stick0_1,
                double wrist0_1,
                double twist0_1,
                double grip0_1) {
            return new LynxArmAngles(
                    m_config,
                    swing0_1,
                    boom0_1,
                    stick0_1,
                    wrist0_1,
                    twist0_1,
                    grip0_1);
        }

        private double fromSwingRad(double swingRad) {
            return m_config.swingCenter - swingRad / m_config.swingScale;
        }

        private double fromBoomRad(double boomRad) {
            return m_config.boomCenter - boomRad / m_config.boomScale;
        }

        private double fromStickRad(double stickRad) {
            return m_config.stickOffset + stickRad / m_config.stickScale;
        }

        private double fromWristRad(double wristRad) {
            return m_config.wristCenter - wristRad / m_config.wristScale;
        }

    }

    private final Config m_config;
    // these are all native servo units
    /** 0.5 is mid, 0.25 is pi/4 left */
    public final double swing;
    /** 0.5 is up, 0.75 is pi/3 back */
    public final double boom;
    /** 0 is parallel to boom, 0.5 is pi/2 down */
    public final double stick;
    /** 0.55 is parallel to stick, 0.3 is pi/4 down */
    public final double wrist;
    /** 0.5 is horizontal */
    public final double twist;
    /** 1 is shut, 0 is open */
    public final double grip;

    private LynxArmAngles(
            Config config,
            double swing0_1,
            double boom0_1,
            double stick0_1,
            double wrist0_1,
            double twist0_1,
            double grip0_1) {
        m_config = config;
        this.swing = check(swing0_1, "swing");
        this.boom = check(boom0_1, "boom");
        this.stick = check(stick0_1, "stick");
        this.wrist = check(wrist0_1, "wrist");
        this.twist = check(twist0_1, "twist");
        this.grip = check(grip0_1, "grip");
    }

    private double check(double x, String name) {
        if (x < 0 || x > 1)
            // throw new IllegalArgumentException(String.format("%s was %f", name, x));
            System.out.printf("%s was %f", name, x);
        return x;
    }

    /**
     * Swing is zero at center, positive counterclockwise.
     */
    public double swingRad() {
        return (m_config.swingCenter - swing) * m_config.swingScale;
    };

    /**
     * Boom straight up is zero, positive forward.
     */
    public double boomRad() {
        return (m_config.boomCenter - boom) * m_config.boomScale;
    };

    /**
     * Stick is **relative to boom,** unlike the arm from 2023.
     * Zero is fully extended, positive is forward.
     */
    public double stickRad() {
        return (m_config.stickOffset + stick) * m_config.stickScale;
    };

    /**
     * Wrist is relative to stick.
     * Zero is fully extended, positive is forward.
     */
    public double wristRad() {
        return (m_config.wristCenter - wrist) * m_config.wristScale;
    };

    /** Used to provide wrist action without moving the rest of the arm. */
    public LynxArmAngles down(double downIncrementRad) {
        return new LynxArmAngles(
                m_config,
                swing,
                boom,
                stick,
                wrist - downIncrementRad / Math.PI,
                twist,
                grip);
    }

    @Override
    public String toString() {
        return "LynxArmAngles [swing=" + swing
                + ", boom=" + boom
                + ", stick=" + stick
                + ", wrist=" + wrist
                + ", twist=" + twist
                + ", grip=" + grip + "]";
    }

}