package org.firstinspires.ftc.teamcode.mechanisms;

public class ShootConstants {
    public static double aprilTagScanTimePerStep_seconds = 0.3;
    public static double aprilTagMoveScanTimePerStep_seconds = 0.3;
    public static double flywheelAccelerationTime_seconds = 1;
    public static double shotDuration_seconds = 2.5;


    static double sizePerDistanceSegment_inches = 144d / 3;
    
    public static double powerFromDistance(double distance) {
        if(distance <= sizePerDistanceSegment_inches) {
            return 0.65;
        } else if (distance <= sizePerDistanceSegment_inches * 2) {
            return 0.75;
        } else {
            return 0.8;
        }
    }


    public static double tiltFromDistance(double distance){
        if(distance <= sizePerDistanceSegment_inches) {
            return 0;
        } else if (distance <= sizePerDistanceSegment_inches * 2) {
            return 0;
        } else {
            return 0;
        }
    }
}
