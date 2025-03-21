package org.team100.lib.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents all the cameras. Some may be mounted on one robot, some on
 * another. Keep this up to date when you move cameras around.
 * Counter-clockwise is positive
 * x is forward, y is left, and z is up
 */
public enum Camera {

    C("10000000a7c673d9",
            new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, -Math.toRadians(10), 0))),
    /**
     * Delta shooter
     */
    SHOOTER("10000000a7a892c0",
            new Transform3d(
                    new Translation3d(-0.1265, 0.0682, 0.612),
                    new Rotation3d(0, Math.toRadians(-25), Math.toRadians(-2)))),

    /**
     * Delta amp-placer
     */
    RIGHTAMP("10000000caeaae82",
            new Transform3d(
                    new Translation3d(-0.1265, -0.1063625, 0.61),
                    new Rotation3d(0, Math.toRadians(-26), Math.toRadians(-63)))),

    /**
     * Delta amp-placer
     */
    LEFTAMP("100000004e0a1fb9",
            new Transform3d(
                    new Translation3d(-0.1265, 0.1532, 0.61),
                    new Rotation3d(0, Math.toRadians(-22), Math.toRadians(59)))),
    /**
     * Delta intake
     */
    GAME_PIECE("1000000013c9c96c",
            new Transform3d(
                    new Translation3d(-0.1265, 0.03, 0.61),
                    new Rotation3d(0, Math.toRadians(31.5), Math.PI))),

    /**
     * Camera bot intake
     */
    GLOBAL_GAME_PIECE("d44649628c20d4d4",
            new Transform3d(
                    new Translation3d(0.183, -0.0889, 0.376),
                    new Rotation3d(Math.toRadians(-2), Math.toRadians(11), 0))),

    /**
     * Right swerve
     */
    SWERVE_RIGHT("47403d5eafe002a9",
            new Transform3d(
                    new Translation3d(-0.261, -0.317, 0.217),
                    new Rotation3d(-0.146, 0.195, -0.508).unaryMinus().plus(new Rotation3d(0, 0, -Math.PI / 2)))),
    // new Rotation3d(0, 0, 0))),

    /**
     * Left swerve
     */
    SWERVE_LEFT("8132c256f63bbb4e",
            new Transform3d(
                    new Translation3d(-0.241, 0.297, 0.207),
                    // new Rotation3d(0, Math.toRadians(-6), Math.toRadians(60)))),
                    new Rotation3d(0.07, 0.147, 0.52).unaryMinus().plus(new Rotation3d(0, 0, Math.PI / 2)))),

    /**
     * Funnel
     */
    FUNNEL("1e5acbaa5a7f9d10",
            new Transform3d(
                    new Translation3d(-0.023, -0.233, 0.902),
                    new Rotation3d(0.087, 0.455, 0.216).unaryMinus().plus(new Rotation3d(0, 0, Math.PI)))),

    /**
     * Coral reef left
     */
    CORAL_LEFT("8ddb2ed6c49a9bce",
            new Transform3d(
                    new Translation3d(0.198, 0.284, 0.811),
                    // new Rotation3d(-0.16, Math.toRadians(40), Math.toRadians(-18)))),
                    new Rotation3d(-0.043, -0.705, 0.254).unaryMinus())),
    // new Rotation3d(0, 0, 0))),

    /**
     * Coral reef right
     */
    CORAL_RIGHT("364f07fb090a3bf7",
            new Transform3d(
                    new Translation3d(0.215, -0.305, 0.79),
                    // new Rotation3d(0.01, -0.83, -0.199).unaryMinus())),
                    // new Rotation3d(0.020, -0.84, -0.195).unaryMinus())),
                    new Rotation3d(0.015, -0.915, -0.170).unaryMinus())),

    TEST1("test1",
            new Transform3d(
                    new Translation3d(),
                    new Rotation3d(0, Math.PI / 4, 0))),

    TEST2("test2",
            new Transform3d(
                    new Translation3d(1, 0, 0),
                    new Rotation3d())),

    TEST3("test3",
            new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, Math.PI / 6, 0))),

    TEST4("test4",
            new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, -Math.PI / 4, 0))),

    UNKNOWN(null, new Transform3d());

    private static Map<String, Camera> cameras = new HashMap<>();
    static {
        for (Camera i : Camera.values()) {
            cameras.put(i.m_serialNumber, i);
        }
    }
    private String m_serialNumber;
    private Transform3d m_Offset;

    private Camera(String serialNumber, Transform3d offset) {
        m_serialNumber = serialNumber;
        m_Offset = offset;
    }

    public static Camera get(String serialNumber) {
        if (cameras.containsKey(serialNumber))
            return cameras.get(serialNumber);
        return UNKNOWN;
    }

    public Transform3d getOffset() {
        return m_Offset;
    }

    public String getSerial() {
        return m_serialNumber;
    }
}
