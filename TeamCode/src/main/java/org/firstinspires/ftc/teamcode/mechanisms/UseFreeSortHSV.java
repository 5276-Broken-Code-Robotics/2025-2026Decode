package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UseFreeSortHSV {

    char pos1;
    char pos2;
    char pos3;

    /*
    g = green
    p = purple
    e = empty


         (pos1)
      (pos2) (pos3)
        |intake|
    */

    Servo arm1;
    Servo arm2;
    Servo arm3;

    RevColorSensorV3 sensor1;
    RevColorSensorV3 sensor2;
    ColorRangeSensor sensor3;

    float sensorGain = 30f;

    float purpleHMaxV3 = 245;
    float purpleHMinV3 = 185;
    float greenHMaxV3 = 175;
    float greenHMinV3 = 145;

    float purpleHMaxV2 = 290;
    float purpleHMinV2 = 180;
    float greenHMaxV2 = 160;
    float greenHMinV2 = 120;




    boolean arm1Shooting = false;
    boolean arm2Shooting = false;
    boolean arm3Shooting = false;



    ElapsedTime shootCD;

    double kickRot = .4;
    double emptyRot = kickRot/2;
    /*

    kickRot = degrees needed to kick ball into turret/270
     */






    public void init(HardwareMap hardwareMap) {


        shootCD = new ElapsedTime();

        shootCD.reset();
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor2");
        sensor3 = hardwareMap.get(ColorRangeSensor.class, "sensor3");

        sensor1.setGain(sensorGain);
        sensor2.setGain(sensorGain);
        sensor3.setGain(sensorGain);
    }


    public void shootArm1() {
        shoot(arm1);
        pos1 = 'e';
    }

    public void shootArm2 () {
        shoot(arm2);
        pos2 = 'e';
    }

    public void shootArm3() {
        shoot(arm3);
        pos3 = 'e';
    }




    public void shoot(Servo arm){
        shootCD.reset();

        if (arm!=arm1 && pos1 == 'e'){arm1.setPosition(emptyRot);}
        if (arm!=arm2 && pos2 == 'e'){arm2.setPosition(emptyRot);}
        if (arm!=arm3 && pos3 == 'e'){arm3.setPosition(emptyRot);}

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

        if(arm1.getPosition() == emptyRot){
            if(shootCD.seconds() >= 0.05f){
                arm1.setPosition(0);
                shootCD.reset();
                arm1Shooting = false;
            }else{
                arm1Shooting = true;
            }
        }

        if(arm2.getPosition() == emptyRot){
        if(shootCD.seconds() >= 0.05f){
            arm2.setPosition(0);
            shootCD.reset();
            arm2Shooting = false;
        }else{
            arm2Shooting = true;

        }
    }

    if(arm3.getPosition() == emptyRot){
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


        float sensor1hue, sensor1sat, sensor1val;
        float sensor2hue, sensor2sat, sensor2val;
        float sensor3hue, sensor3sat, sensor3val;

        sensor1hue = JavaUtil.colorToHue(sensor1Colors.toColor());
        sensor1sat = JavaUtil.colorToSaturation(sensor1Colors.toColor());
        sensor1val = JavaUtil.colorToValue(sensor1Colors.toColor());

        sensor2hue = JavaUtil.colorToHue(sensor2Colors.toColor());
        sensor2sat = JavaUtil.colorToSaturation(sensor2Colors.toColor());
        sensor2val = JavaUtil.colorToValue(sensor2Colors.toColor());

        sensor3hue = JavaUtil.colorToHue(sensor3Colors.toColor());
        sensor3sat = JavaUtil.colorToSaturation(sensor3Colors.toColor());
        sensor3val = JavaUtil.colorToValue(sensor3Colors.toColor());



        telemetry.addLine("Sensor 1");
        telemetry.addData("hue", sensor1hue);
        //telemetry.addData("sat", sensor1sat);
        //telemetry.addData("value", sensor1val);
        telemetry.addData("Pos1", pos1);


        telemetry.addLine("Sensor 2");
        telemetry.addData("hue", sensor2hue);
        //telemetry.addData("sat", sensor2sat);
        //telemetry.addData("value", sensor2val);
        telemetry.addData("Pos2", pos2);

        telemetry.addLine("Sensor 3");
        telemetry.addData("hue", sensor3hue);
        //telemetry.addData("sat", sensor3sat);
        //telemetry.addData("value", sensor3val);
        telemetry.addData("Pos3", pos3);



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

        if (purpleHMaxV2 > sensor3hue && sensor3hue > purpleHMinV2){
            pos3 = 'p';
        }
        else if (greenHMaxV2 > sensor3hue && sensor3hue > greenHMinV2){
            pos3 = 'g';
        }
        else{
            pos3 = 'e';
        }

    }

}
