// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldConstants {
    public enum FieldSector {
        S1,
        S2,
        S3,
        S4,
        S5,
        S6
    }

    public static FieldConstants.FieldSector getSector(Pose2d pose){
        Translation2d target = FieldConstants.getReefCenter().minus(pose.getTranslation());
        Rotation2d targetAngle = target.getAngle();
        double angle = targetAngle.getDegrees();

        System.out.println("ANGLEEEE" + angle);

        if(angle >= -30 && angle <= 30){
            return FieldSector.S1;
        } else if(angle >= -90 && angle <-30){
            return FieldSector.S2;
        } else if(angle >= -150 && angle <= -90){
            return FieldSector.S3;
        } else if(angle >= 150 || angle <= -150){
            return FieldSector.S4;
        } else if(angle >= 90 && angle <= 150){
            return FieldSector.S5;
        } else if(angle >= 30 && angle <= 90){
            return FieldSector.S6;
        } else {
            return FieldSector.S1;
        }
    }

    public static Translation2d getReefCenter(){ //blue
        return new Translation2d(4.508405, 4.038690);
    }

    public static double getOrbitRadius(){
        return 2.7;
    }
}
