package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class FreeSort {

    char pos1 = 'g';
    char pos2 = 'p';
    char pos3 = 'p';

    //    |intake|
    //  (pos2) (pos3)
    //     (pos1)
    //

    Servo arm1;
    Servo arm2;
    Servo arm3;

    NormalizedColorSensor sensor1;
    NormalizedColorSensor sensor2;
    NormalizedColorSensor sensor3;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }







    public void init(HardwareMap hardwareMap) {

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        sensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor1");
        sensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor2");
        sensor3 = hardwareMap.get(NormalizedColorSensor.class, "sensor3");

    }

    public DetectedColor getDetectedColor() {
        NormalizedRGBA colors = sensor1.getNormalizedColors();
    }

    public void shootAll(){

    }

}
