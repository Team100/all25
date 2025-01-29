// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldConstants {
    public enum FieldSector {
        AB,
        CD,
        EF,
        GH,
        IJ,
        KL
    }

    public enum ReefPoint {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
        CENTER
    }

    public static FieldConstants.FieldSector getSector(Pose2d pose){
        Translation2d target = FieldConstants.getReefCenter().minus(pose.getTranslation());
        Rotation2d targetAngle = target.getAngle();
        double angle = targetAngle.getDegrees();

        System.out.println("ANGLEEEE" + angle);

        if(angle >= -30 && angle <= 30){
            return FieldSector.AB;
        } else if(angle >= -90 && angle <-30){
            return FieldSector.KL;
        } else if(angle >= -150 && angle <= -90){
            return FieldSector.IJ;
        } else if(angle >= 150 || angle <= -150){
            return FieldSector.GH;
        } else if(angle >= 90 && angle <= 150){
            return FieldSector.EF;
        } else if(angle >= 30 && angle <= 90){
            return FieldSector.CD;
        } else {
            return FieldSector.AB;
        }
    }

    public static Rotation2d getSectorAngle(FieldSector sector){
        switch(sector){
            case AB:
                return Rotation2d.fromDegrees(180);
            case CD:
                return Rotation2d.fromDegrees(-120);
            case EF:
                return Rotation2d.fromDegrees(-60);
            case GH:
                return Rotation2d.fromDegrees(0);
            case IJ:
                return Rotation2d.fromDegrees(60);
            case KL:
                return Rotation2d.fromDegrees(120);
            default:
                return Rotation2d.fromDegrees(0);
        }
    }

    public static Rotation2d angleToReefCenter(Pose2d pose){
        Translation2d target = FieldConstants.getReefCenter().minus(pose.getTranslation());
        Rotation2d targetAngle = target.getAngle();
        return targetAngle;
    }

    public static Translation2d getReefCenter(){ //blue
        return new Translation2d(4.508405, 4.038690);
    }

    public static double getOrbitWaypointRadius(){
        return 2.7;
    }

    public static double getOrbitDestinationRadius(){
        return 1.8;
    }

    public static double getReefOffset(){
        return 0.1524;
    }

    public static Translation2d getOrbitWaypoint(Rotation2d angle){
        Translation2d reefCenter = getReefCenter();
        double x = reefCenter.getX() + getOrbitWaypointRadius() * angle.getCos();
        double y = reefCenter.getY() + getOrbitWaypointRadius() * angle.getSin();

        return new Translation2d(x, y);

    }

    public static Translation2d getOrbitWaypoint(FieldSector sector){
        Translation2d reefCenter = getReefCenter();
        double x = reefCenter.getX() + getOrbitWaypointRadius() * getSectorAngle(sector).getCos();
        double y = reefCenter.getY() + getOrbitWaypointRadius() * getSectorAngle(sector).getSin();

        return new Translation2d(x, y);

    }

    public static Translation2d getOrbitDestination(FieldSector destinationSector, ReefPoint destinationPoint){
        Translation2d reefCenter = getReefCenter();
        Rotation2d sectorAngle = getSectorAngle(destinationSector);

        

        double x = reefCenter.getX() + getOrbitDestinationRadius() * sectorAngle.getCos();
        double y = reefCenter.getY() + getOrbitDestinationRadius() * sectorAngle.getSin();


        double dx = 0;
        double dy = 0;
        
        dx = (getReefOffset() * Math.cos(90 + sectorAngle.getDegrees()));
        dy = (getReefOffset() * Math.sin(90 + sectorAngle.getDegrees()));

        
        switch(destinationPoint){
            case A, C, E, G, I, K:
                return new Translation2d(x += dx, y += dy);
            case B, D, F, H, J, L:
                return new Translation2d(x -= dx, y -= dy);
            case CENTER:
                return new Translation2d(x, y);
        
        }

        return new Translation2d(x, y);

    }
}
