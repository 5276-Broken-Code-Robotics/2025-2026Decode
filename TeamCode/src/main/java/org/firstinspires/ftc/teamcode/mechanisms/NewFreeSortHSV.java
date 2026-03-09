package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class NewFreeSortHSV {

    char pos1;
    char pos2;
    char pos3;

    Servo arm1;
    Servo arm2;
    Servo arm3;

    RevColorSensorV3 sensor1;
    RevColorSensorV3 sensor2;
    RevColorSensorV3 sensor3;

    float sensorGain = 15f;

    float purpleHMaxV3 = 245;
    float purpleHMinV3 = 185;
    float greenHMaxV3 = 175;
    float greenHMinV3 = 155;



//TODO ADJUST THESE VALUES ^^^^^^^^^^^^^^^^^ ----- VENUE SPECIFIC


    boolean arm1Busy = false;
    boolean arm2Busy = false;
    boolean arm3Busy = false;

    ElapsedTime shootCD1;
    ElapsedTime shootCD2;
    ElapsedTime shootCD3;

    Float shootCDTime = .2f;

    double kickRot = .4;
    double emptyRot = .25;



    public void init(HardwareMap hardwareMap) {

        shootCD1 = new ElapsedTime();
        shootCD2 = new ElapsedTime();
        shootCD3 = new ElapsedTime();

        shootCD1.reset();
        shootCD2.reset();
        shootCD3.reset();

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        arm1.scaleRange(0.03, 1);
        //arm2.scaleRange(0.03, 1);
        arm3.scaleRange(0.03, 1);

        sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor2");
        sensor3 = hardwareMap.get(RevColorSensorV3.class, "sensor3");

        sensor1.setGain(sensorGain);
        sensor2.setGain(sensorGain);
        sensor3.setGain(sensorGain);

        arm1.setPosition(0);
        arm2.setPosition(0);
        arm3.setPosition(0);

    }

    public void shoot(int arm){

        if (arm==1 && pos1 != 'e' && !arm1Busy){
            arm1.setPosition(kickRot);
            shootCD1.reset();
            block(1);

        }
        if (arm==2 && pos2 != 'e' && !arm2Busy){
            arm2.setPosition(kickRot);
            shootCD2.reset();
            block(2);

        }
        if (arm==3 && pos3 != 'e' && !arm3Busy){
            arm3.setPosition(kickRot);
            shootCD3.reset();
            block(3);

        }


    }

    private void block (int arm) {
        if (arm != 1 && pos1 == 'e'){
            arm1.setPosition(emptyRot);
            shootCD1.reset();
        }
        if (arm != 2 && pos2 == 'e'){
            arm2.setPosition(emptyRot);
            shootCD2.reset();
        }
        if (arm != 3 && pos3 == 'e'){
            arm3.setPosition(emptyRot);
            shootCD3.reset();
        }
    }

    public void shootPattern (char pattern) {
        if (pattern == 'p') {
            if(pos1 == 'p') {shoot(1); return;}
            if(pos2 == 'p') {shoot(2); return;}
            if(pos3 == 'p') {shoot(3);}
        }
        if (pattern == 'g') {
            if(pos1 == 'g') {shoot(1); return;}
            if(pos2 == 'g') {shoot(2); return;}
            if(pos3 == 'g') {shoot(3);}
        }
        if (pattern == 'a') {
            shoot(1);
            shoot(2);
            shoot(3);
        }

    }

    public void loop () {
        if (shootCD1.seconds() >= shootCDTime) {
            arm1.setPosition(0);
        }
        if (shootCD2.seconds() >= shootCDTime) {
            arm2.setPosition(0);
        }
        if (shootCD3.seconds() >= shootCDTime) {
            arm3.setPosition(0);
        }

    }


    public void updateColors (Telemetry telemetry) {
        NormalizedRGBA sensor1Colors = sensor1.getNormalizedColors();
        NormalizedRGBA sensor2Colors = sensor2.getNormalizedColors();
        NormalizedRGBA sensor3Colors = sensor3.getNormalizedColors();


        float sensor1hue;
        float sensor2hue;
        float sensor3hue;

        sensor1hue = JavaUtil.colorToHue(sensor1Colors.toColor());
        sensor2hue = JavaUtil.colorToHue(sensor2Colors.toColor());
        sensor3hue = JavaUtil.colorToHue(sensor3Colors.toColor());



        telemetry.addLine("Sensor 1");
        telemetry.addData("hue", sensor1hue);
        telemetry.addData("Pos1", pos1);
        telemetry.addData("Arm1", arm1.getPosition());
        telemetry.addData("CD1", shootCD1.seconds());






        telemetry.addLine("Sensor 2");
        telemetry.addData("hue", sensor2hue);
        telemetry.addData("Pos2", pos2);
        telemetry.addData("Arm2", arm2.getPosition());
        telemetry.addData("CD2", shootCD2.seconds());


        telemetry.addLine("Sensor 3");
        telemetry.addData("hue", sensor3hue);
        telemetry.addData("Pos3", pos3);
        telemetry.addData("Arm3", arm3.getPosition());
        telemetry.addData("CD3", shootCD3.seconds());


        if (purpleHMaxV3 > sensor1hue && sensor1hue > purpleHMinV3){
            pos1 = 'p';
        }
        else if (greenHMaxV3 > sensor1hue && sensor1hue > greenHMinV3){
            pos1 = 'g';
        }
        else{
            pos1 = 'e';
        }


        if (purpleHMaxV3 > sensor2hue && sensor2hue > purpleHMinV3){
            pos2 = 'p';
        }
        else if (greenHMaxV3 > sensor2hue && sensor2hue > greenHMinV3){
            pos2 = 'g';
        }
        else{
            pos2 = 'e';
        }

        if (purpleHMaxV3 > sensor3hue && sensor3hue > purpleHMinV3){
            pos3 = 'p';
        }
        else if (greenHMaxV3 > sensor3hue && sensor3hue > greenHMinV3){
            pos3 = 'g';
        }
        else {
            pos3 = 'e';
        }

    }
}
