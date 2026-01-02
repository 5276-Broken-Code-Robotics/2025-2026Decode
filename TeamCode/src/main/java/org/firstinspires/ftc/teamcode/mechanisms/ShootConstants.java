package org.firstinspires.ftc.teamcode.mechanisms;

public class ShootConstants {
    public static double aprilTagScanTimePerStep_seconds = 0.4;
    public static double aprilTagMoveScanTimePerStep_seconds = 0.4;
    public static double flywheelAccelerationTime_seconds = 1;
    public static double shotDuration_seconds = 2.5;
    
    public static double powerFromDistance(double distance) {
        if(distance <= 3) {
            return 0.575;
        } else if (distance <= 6) {
            return 0.63;
        } else {
            return 0.70;
        }
    }


    public static double tiltFromDistance(double distance){
        return 0;
    }
}
