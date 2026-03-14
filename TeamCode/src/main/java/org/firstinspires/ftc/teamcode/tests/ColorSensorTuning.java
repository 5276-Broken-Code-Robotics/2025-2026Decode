package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(group = "tuning")
public class ColorSensorTuning extends OpMode {

    RevColorSensorV3 sensor1;
    RevColorSensorV3 sensor2;
    RevColorSensorV3 sensor3;

    float sensor1hue;
    float sensor2hue;
    float sensor3hue;

    float sensorGain = 55f;

    float purpleHMinV3 = 170;


    float greenSMinV3 = 0.55f;


    char pos1;
    char pos2;
    char pos3;



    @Override
    public void init() {

        sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor2");
        sensor3 = hardwareMap.get(RevColorSensorV3.class, "sensor3");

        sensor1.setGain(sensorGain);
        sensor2.setGain(sensorGain);
        sensor3.setGain(sensorGain);


    }

    @Override
    public void loop() {

        NormalizedRGBA sensor1Colors = sensor1.getNormalizedColors();
        NormalizedRGBA sensor2Colors = sensor2.getNormalizedColors();
        NormalizedRGBA sensor3Colors = sensor3.getNormalizedColors();

        sensor1hue = JavaUtil.colorToHue(sensor1Colors.toColor());
        sensor2hue = JavaUtil.colorToHue(sensor2Colors.toColor());
        sensor3hue = JavaUtil.colorToHue(sensor3Colors.toColor());




        if (sensor1hue > purpleHMinV3){
            pos1 = 'p';
        }
        else if (JavaUtil.colorToSaturation(sensor1Colors.toColor()) > greenSMinV3){
            pos1 = 'g';
        }
        else {
            pos1 = 'e';
        }

        if (sensor2hue > purpleHMinV3){
            pos2 = 'p';
        }
        else if (JavaUtil.colorToSaturation(sensor2Colors.toColor()) > greenSMinV3){
            pos2 = 'g';
        }
        else {
            pos2 = 'e';
        }

        if (sensor3hue > purpleHMinV3){
            pos3 = 'p';
        }
        else if (JavaUtil.colorToSaturation(sensor3Colors.toColor()) > greenSMinV3){
            pos3 = 'g';
        }
        else {
            pos3 = 'e';
        }




        telemetry.addLine("=== Sensor 1 =========");
        telemetry.addData("H", sensor1hue);
        telemetry.addData("S", JavaUtil.colorToSaturation(sensor1Colors.toColor()));
        telemetry.addData("V", JavaUtil.colorToValue(sensor1Colors.toColor()));

        //telemetry.addData("dist", sensor1.getDistance(DistanceUnit.MM));
        telemetry.addData("Pos1", pos1);

        telemetry.addLine("=== Sensor 2 =========");
        telemetry.addData("H", sensor2hue);
        telemetry.addData("S", JavaUtil.colorToSaturation(sensor2Colors.toColor()));
        telemetry.addData("V", JavaUtil.colorToValue(sensor2Colors.toColor()));
        //telemetry.addData("dist", sensor2.getDistance(DistanceUnit.MM));
        telemetry.addData("Pos2", pos2);


        telemetry.addLine("=== Sensor 3 =========");
        telemetry.addData("H", sensor3hue);
        telemetry.addData("S", JavaUtil.colorToSaturation(sensor3Colors.toColor()));
        telemetry.addData("V", JavaUtil.colorToValue(sensor3Colors.toColor()));
        //telemetry.addData("dist", sensor3.getDistance(DistanceUnit.MM));
        telemetry.addData("Pos3", pos3);

        telemetry.update();


    }
}
