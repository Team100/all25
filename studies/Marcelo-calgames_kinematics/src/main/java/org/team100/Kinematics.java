package org.team100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Kinematics {
    private final double m_armLength;
    private final double m_wristLength;
    private final double m_elevatorHeight;

    public Kinematics(double armLength, double wristLength, double elevatorHeight) {
        m_armLength = armLength;
        m_wristLength = wristLength;
        m_elevatorHeight = elevatorHeight;
    }

    public Pose2d forward(Config config) {
        // forward kinematics
        double pivotHeight = m_elevatorHeight + config.pivotHeightM();
        double WristX = m_armLength * Math.cos(config.pivotAngleRad()) + m_wristLength * Math.cos(config.pivotAngleRad() + config.wristAngleRad());
        double WristY = pivotHeight + m_armLength * Math.sin(config.pivotAngleRad()) + m_wristLength * Math.sin(config.pivotAngleRad() + config.wristAngleRad());
        double FinalX = WristX+ m_wristLength * Math.cos(config.pivotAngleRad() + config.wristAngleRad());
        double FinalY = WristY + m_wristLength * Math.sin(config.pivotAngleRad() + config.wristAngleRad());
        return new Pose2d(FinalX, FinalY, new Rotation2d(config.pivotAngleRad() + config.wristAngleRad()));
    }

    public Config inverse(Pose2d pose) {
        // inverse kinematics
        double x = pose.getX();
        System.out.println("X " + x);
        double y = pose.getY();
        System.out.println("Y " + y);
        double Angel = pose.getRotation().getRadians();
        System.out.println("Angle " + Angel);
        double WristX = x - m_wristLength * Math.cos(Angel);
        double WristY = y - m_wristLength * Math.sin(Angel + Math.toRadians(90));
        System.out.println("WristX: " + WristX);
        System.out.println("WristY: " + WristY);
        double GripX = WristX;
        double GripY = WristY - m_elevatorHeight;
        System.out.println("GripX: " + GripX);
        System.out.println("GripY: " + GripY);
        double distance = Math.sqrt(GripX * GripX + GripY * GripY);
        double pivotTOTArgetAngle = Math.atan2(GripY, GripX);
        System.out.println("Distance: " + distance);
        System.out.println("Pivot to target angle: " + pivotTOTArgetAngle);
        double cosbendAngle = (distance * distance + m_armLength * m_armLength - m_wristLength * m_wristLength) / (2 * distance * m_armLength);
        cosbendAngle = Math.max(-1, Math.min(1, cosbendAngle));
        System.out.println("Cosine bend angle: " + cosbendAngle);
        double bendAngle = Math.acos(cosbendAngle);
        System.out.println("Bend angle: " + bendAngle);
        double secondPivotAngle = pivotTOTArgetAngle + bendAngle;
        System.out.println("Second pivot angle: " + secondPivotAngle);
        double wristAngle = Angel - secondPivotAngle;
        System.out.println("Wrist angle: " + wristAngle);
        double pivotHeight = m_elevatorHeight;
        System.out.println("Pivot height: " + pivotHeight);
    

        return new Config(secondPivotAngle, wristAngle, pivotHeight);
    }

}