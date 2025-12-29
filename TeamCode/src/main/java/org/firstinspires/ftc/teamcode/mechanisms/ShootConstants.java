package org.firstinspires.ftc.teamcode.mechanisms;

public class ShootConstants {
    public static double aprilTagScanTimePerStep_seconds = 1;
    public static double flywheelAccelerationTime_seconds = 1;
    public static double timeToResetTransferServo_seconds = 1;

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

    public static String[] shootStates = {
            "looking_for_april_tag",
            "found_tag_orienting",
            "shoot",
            "cleanup_shoot"
    };
}
