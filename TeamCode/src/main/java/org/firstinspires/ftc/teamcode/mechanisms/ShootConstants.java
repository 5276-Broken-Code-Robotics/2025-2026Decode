package org.firstinspires.ftc.teamcode.mechanisms;

public class ShootConstants {
    public static double aprilTagScanTimePerStep_seconds = 0.4;
    public static double aprilTagMoveScanTimePerStep_seconds = 0.4;
    public static double flywheelAccelerationTime_seconds = 1;
    public static double shotDuration_seconds = 4;
    
    public static double powerFromDistance(double distance) {
        if(distance <= 36) {
            return 0.58;
        } else if (distance <= 72) {
            return 0.63;
        } else if(distance <= 120){
            return 0.68;
        }else{
            return 0.7;
        }
    }


    public static double tiltFromDistance(double distance){


        if(distance <= 36) {
            return 0;
        } else if (distance <= 72) {

            double val = -0.0015625 * distance * distance + 0.175*distance -4.5;
            if(val > 0)return val;

            return 0;


        } else if(distance <= 120){
            double val = -0.00037037*distance*distance + 0.0591667 * distance - 1.76667;


            if(val > 0) return val;

            return 0;
        }else{
            return 0.275;
        }
    }
}
