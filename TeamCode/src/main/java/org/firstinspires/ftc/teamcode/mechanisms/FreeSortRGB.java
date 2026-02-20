package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FreeSortRGB {

    char pos1 = 'g';
    char pos2 = 'p';
    char pos3 = 'p';

    /*
    g = green
    p = purple
    e = empty

    always preload like this:
               (g)
             (p)(p)
             |intake|

     */

    //     (pos1)
    //  (pos2) (pos3)
    //    |intake|
    //

    Servo arm1;
    Servo arm2;
    Servo arm3;

    NormalizedColorSensor sensor1;
    NormalizedColorSensor sensor2;
    NormalizedColorSensor sensor3;

    float sensorGain = 30f;

    float purpleRV3 = 1.0f;
    float purpleGV3 = 1.0f;
    float purpleBV3 = 1.0f;

    float greenRV3 = 1.0f;
    float greenGV3 = 0.7f;
    float greenBV3 = 0.21f;

    float purpleRV2 = 1.0f;
    float purpleGV2 = 1.0f;
    float purpleBV2 = 1.0f;

    float greenRV2 = 1.0f;
    float greenGV2 = 1.0f;
    float greenBV2 = 1.0f;

    boolean arm1Shooting = false;
    boolean arm2Shooting = false;
    boolean arm3Shooting = false;


    ElapsedTime shootCD;
    //TODO test to find rgb values
    double kickRot = 1;
    /*
    TODO calculate kickRot once cad done
    kickRot = degrees needed to kick ball into turret/270
     */






    public void init(HardwareMap hardwareMap) {


        shootCD = new ElapsedTime();

        shootCD.reset();
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        sensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor1");
        sensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor2");
        sensor3 = hardwareMap.get(NormalizedColorSensor.class, "sensor3");

        sensor1.setGain(sensorGain);
        sensor2.setGain(sensorGain);
        sensor3.setGain(sensorGain);
    }



    public void shootAll(){
        if (pos1 != 'e' && !arm1Shooting) {
            shoot(arm1); pos1 = 'e';
        }
        if (pos2 != 'e' && !arm2Shooting) {
            shoot(arm2); pos2 = 'e';
        }
        if (pos3 != 'e' && !arm3Shooting) {
            shoot(arm3); pos3 = 'e';
        }
    }
    public void shootGreen(){

        if (pos1 == 'g' && !arm1Shooting) {
            shoot(arm1); pos1 = 'e';
        }
        else if (pos2 == 'g' && !arm2Shooting) {
            shoot(arm2); pos2 = 'e';
        }
        else if (pos3 == 'g' && !arm3Shooting) {
            shoot(arm3); pos3 = 'e';
        }
    }
    public void shootPurple(){

        if (pos1 == 'p' && !arm1Shooting) {
            shoot(arm1); pos1 = 'e';

        }
        else if (pos2 == 'p' && !arm2Shooting) {
            shoot(arm2); pos2 = 'e';
        }
        else if (pos3 == 'p' && !arm3Shooting) {
            shoot(arm3); pos3 = 'e';
        }
    }

    public void shoot(Servo arm){
        shootCD.reset();
        arm.setPosition(kickRot);

    }


    public void loop(){

        if(arm1.getPosition() == kickRot){
            if(shootCD.seconds() >= 0.05f){
                arm1.setPosition(0);
                shootCD.reset();
                arm1Shooting = false;
            }else{
                arm1Shooting = true;
            }
        }

        if(arm2.getPosition() == kickRot){
            if(shootCD.seconds() >= 0.05f){
                arm2.setPosition(0);
                shootCD.reset();
                arm2Shooting = false;
            }else{
                arm2Shooting = true;

            }
        }

        if(arm3.getPosition() == kickRot){
            if(shootCD.seconds() >= 0.05f){
                arm3.setPosition(0);
                shootCD.reset();
                arm3Shooting = false;
            }else{
                arm3Shooting = true;
            }
        }
    }

    public void updateColors(Telemetry telemetry) {
        NormalizedRGBA sensor1Colors = sensor1.getNormalizedColors();
        NormalizedRGBA sensor2Colors = sensor2.getNormalizedColors();
        NormalizedRGBA sensor3Colors = sensor3.getNormalizedColors();

        float pos1red, pos1green, pos1blue;
        float pos2red, pos2green, pos2blue;
        float pos3red, pos3green, pos3blue;

        pos1red = sensor1Colors.red / sensor1Colors.alpha;
        pos1green = sensor1Colors.green / sensor1Colors.alpha;
        pos1blue = sensor1Colors.blue / sensor1Colors.alpha;

        pos2red = sensor2Colors.red / sensor2Colors.alpha;
        pos2green = sensor2Colors.green / sensor2Colors.alpha;
        pos2blue = sensor2Colors.blue / sensor2Colors.alpha;

        pos3red = sensor3Colors.red / sensor3Colors.alpha;
        pos3green = sensor3Colors.green / sensor3Colors.alpha;
        pos3blue = sensor3Colors.blue / sensor3Colors.alpha;



        telemetry.addLine("Sensor 1");
        telemetry.addData("red", pos1red);
        telemetry.addData("green", pos1green);
        telemetry.addData("blue", pos1blue);
        telemetry.addData("Pos1", pos1);

        telemetry.addLine("Sensor 2");
        telemetry.addData("red", pos2red);
        telemetry.addData("green", pos2green);
        telemetry.addData("blue", pos2blue);
        telemetry.addData("Pos2", pos2);

        telemetry.addLine("Sensor 3");
        telemetry.addData("red", pos3red);
        telemetry.addData("green", pos3green);
        telemetry.addData("blue", pos3blue);
        telemetry.addData("Pos3", pos3);

        //TODO assign color values to certain balls for certain colors + change from > to < if needed

        if (pos1red > purpleRV3 && pos1green > purpleGV3 && pos1blue > purpleBV3){
            pos1 = 'p';
        }
        else if (pos1red > greenRV3 && pos1green > greenGV3 && pos1blue > greenBV3){
            pos1 = 'g';
        }
        else{
            pos1 = 'e';
        }


        if (pos2red > purpleRV3 && pos2green > purpleGV3 && pos2blue > purpleBV3){
            pos2 = 'p';
        }
        else if (pos2red > greenRV3 && pos2green > greenGV3 && pos2blue > greenBV3){
            pos2 = 'g';
        }
        else{
            pos2 = 'e';
        }

        if (pos3red > purpleRV3 && pos3green > purpleGV3 && pos3blue > purpleBV3){
            pos3 = 'p';
        }
        else if (pos3red > greenRV3 && pos3green > greenGV3 && pos3blue > greenBV3){
            pos3 = 'g';
        }
        else{
            pos3 = 'e';
        }

    }

}
