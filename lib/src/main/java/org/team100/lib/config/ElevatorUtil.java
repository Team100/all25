// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.config;

/** Add your docs here. */
public class ElevatorUtil {
    public enum ScoringPosition {
        L1(20),
        L2(20),
        L3(20),
        L4(20),
        NONE(0);
    
        private final double value; // Field to store the number
    
        ScoringPosition(double value) {
            this.value = value;
        }
    
        public double getValue() {
            return value;
        }
    
        public static ScoringPosition fromValue(double value) {
            for (ScoringPosition position : ScoringPosition.values()) {
                if (position.getValue() == value) {
                    return position;
                }
            }
            throw new IllegalArgumentException("Invalid scoring position value: " + value);
        }
    }
}
