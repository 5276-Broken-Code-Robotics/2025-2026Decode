package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FreeSort {

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

    float sensorGain = 10.0f;



    float purpleR = 1.0f;
    float purpleG = 1.0f;
    float purpleB = 1.0f;

    float greenR = 1.0f;
    float greenG = 1.0f;
    float greenB = 1.0f;

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

    public void shootPattern(char[] colorList){
        for (int i=0; i<3;i++){
            if(colorList[i]=='p'){
                shootPurple();
            }else{
                shootGreen();
            }
        }
    }
    void loop(){
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

        telemetry.addLine("Sensor 1");
        telemetry.addData("red", sensor1Colors.red);
        telemetry.addData("green", sensor1Colors.green);
        telemetry.addData("blue", sensor1Colors.blue);
        telemetry.addData("Pos1", pos1);

        telemetry.addLine("Sensor 2");
        telemetry.addData("red", sensor2Colors.red);
        telemetry.addData("green", sensor2Colors.green);
        telemetry.addData("blue", sensor2Colors.blue);
        telemetry.addData("Pos2", pos2);

        telemetry.addLine("Sensor 3");
        telemetry.addData("red", sensor3Colors.red);
        telemetry.addData("green", sensor3Colors.green);
        telemetry.addData("blue", sensor3Colors.blue);
        telemetry.addData("Pos3", pos3);

        //TODO assign color values to certain balls for certain colors + change from > to < if needed

        if (sensor1Colors.red > purpleR && sensor1Colors.green > purpleG && sensor1Colors.blue > purpleB){
            pos1 = 'p';
        }
        else if (sensor1Colors.red > greenR && sensor1Colors.green > greenG && sensor1Colors.blue > greenB){
            pos1 = 'g';
        }
        else{
            pos1 = 'e';
        }


        if (sensor2Colors.red > purpleR && sensor2Colors.green > purpleG && sensor2Colors.blue > purpleB){
            pos2 = 'p';
        }
        else if (sensor2Colors.red > greenR && sensor2Colors.green > greenG && sensor2Colors.blue > greenB){
            pos2 = 'g';
        }
        else{
            pos2 = 'e';
        }

        if (sensor3Colors.red > purpleR && sensor3Colors.green > purpleG && sensor3Colors.blue > purpleB){
            pos3 = 'p';
        }
        else if (sensor3Colors.red > greenR && sensor3Colors.green > greenG && sensor3Colors.blue > greenB){
            pos3 = 'g';
        }
        else{
            pos3 = 'e';
        }

    }

}
