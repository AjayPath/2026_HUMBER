// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Variables {
    public static class drive {
        public static double robotHeading;
    }

    public static class intake {
        public static double intakeRPS = 0;
    }   
    public static class feeder {
        public static double feederRPS = 0;
    }

    public static class floor {
        public static double floorRPS = 0;
    }

    public static class shooter {
        public static double shooterRPS = 0;
    }

    public static class pivot {
        public static double pivotPosition = 0;
    }

    public static class limelight {
        public static double tID = 0;
        public static double tA = 0;
        public static double tX = 0;
        public static double tY = 0;
        public static boolean hasValidTarget = false;

        public static double distanceMeters = 0;
        public static double turnAngle = 0;
        public static double shooterRPS = 0;
    }
}