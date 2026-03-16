package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FinalFreeSortHSV {

    public char pos1;
    public char pos2;
    public char pos3;

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
    RevColorSensorV3 sensor3;

    double stuckStartDur = 0f;



    ElapsedTime shootcd;



    ElapsedTime shootWaitTimer;


    boolean arm1Shooting = false;
    boolean arm2Shooting = false;
    boolean arm3Shooting = false;



    ElapsedTime shootCD;

    public boolean armsMoving = false;

    double kickRot = .4;
    double emptyRot = 0.25;

    public int shootnum;
    /*

    kickRot = degrees needed to kick ball into turret/270
     */




    private Telemetry telem;




    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        incrementCD = new ElapsedTime();

        TMa1 = false;
        TMa2 = false;

        TMa3 = false;
        stuckStartCD = new ElapsedTime();
        shootWaitTimer = new ElapsedTime();
        shootCD = new ElapsedTime();




        telem = telemetry;
        shootWaitTimer.reset();
        shootCD.reset();
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor2");
        sensor3 = hardwareMap.get(RevColorSensorV3.class, "sensor3");

        sensor1.setGain(sensorGain);
        sensor2.setGain(sensorGain);
        sensor3.setGain(sensorGain);
        arm1CD = new ElapsedTime();
        arm2CD = new ElapsedTime();
        arm3CD = new ElapsedTime();

    }




    public boolean shooting = false;

    boolean arm1Stuck = false;
    boolean arm2Stuck = false;
    boolean arm3Stuck = false;






    ElapsedTime stuckStartCD;
    public int emptyCount(){
        int count=0;
        if(pos1=='e'){
            count+=1;
        }
        if(pos2=='e'){
            count+=1;
        }
        if(pos3=='e'){
            count+=1;
        }
        return count;
    }
    public void shoot(int armnum){


        shooting = true;
        if (armnum!=1 && pos1 == 'e'){
            arm1Stuck = true;


            arm1.setPosition(emptyRot);
            stuckStartCD.reset();
        }
        if (armnum!=2 && pos2 == 'e'){
            arm2Stuck = true;
            arm2.setPosition(emptyRot);
            stuckStartCD.reset();

        }
        if (armnum!=3 && pos3 == 'e'){
            arm3Stuck = true;
            arm3.setPosition(emptyRot);
            stuckStartCD.reset();

        }


        if(armnum == 1)
        {
            arm1Shooting = true;
        }

        if(armnum == 2)
        {
            arm2Shooting = true;


        }
        if(armnum == 3)
        {
            arm3Shooting = true;
        }
    }



    ElapsedTime arm1CD;
    ElapsedTime arm2CD;
    ElapsedTime arm3CD;



    boolean TMa1 = false;
    boolean TMa2= false;
    boolean TMa3 = false;


    ElapsedTime incrementCD;

    int dex = 0;

    public double shootDur = 0.3;

    int num = 0;
    public void loop(){
        updateColors();

        if(!arm1Shooting && !arm1Stuck){
            arm1.setPosition(0);
        }
        if(!arm2Shooting && !arm2Stuck){
            arm2.setPosition(0);
        }
        if(!arm3Shooting && !arm3Stuck){
            arm3.setPosition(0);
        }


        /*
        telem.addData("A1Shooting:", arm1Shooting);
        telem.addData("A1Stuck:", arm1Stuck);


        telem.addData("A2Shooting", arm2Shooting);
        telem.addData("A2Stuck:", arm2Stuck);

        telem.addData("A3Shooting", arm3Shooting);
        telem.addData("A3Stuck:", arm3Stuck);

        telem.addData("A1 shot yet", TMa1);
        telem.addData("A2 shot yet", TMa2);
        telem.addData("A3 shot yet", TMa3);




        telem.addData("A1 pos", arm1.getPosition() );
        telem.addData("A2 pos", arm2.getPosition() );
        telem.addData("A3 pos", arm3.getPosition() );



        telem.addData("Shooting : ", shooting);
        telem.addData("NUM : ", num);

        telem.addData("P1 : ", pos1);
        telem.addData("P2 : ", pos2);
        telem.addData("P3 : ", pos3);

         */

        num++;



        if(!arm1Shooting && !arm2Shooting && !arm3Shooting){
            arm1Stuck = false;
            arm2Stuck = false;
            arm3Stuck = false;
            shooting = false;
        }



        if(arm1Stuck)arm1.setPosition(emptyRot);
        if(arm2Stuck)arm2.setPosition(emptyRot);
        if(arm3Stuck)arm3.setPosition(emptyRot);


        if(!arm1Shooting){
            arm1CD.reset();
        }else{
            TMa1 = true;
            if(arm1CD.seconds() > shootDur + stuckStartDur){
                arm1.setPosition(0);

                arm1Shooting = false;
                shooting = false;
                arm1Stuck = false;
                arm2Stuck = false;
                arm3Stuck = false;
                stuckStartCD.reset();

            }else{
                if(stuckStartCD.seconds() > stuckStartDur)arm1.setPosition(kickRot);
            }
        }




        if(!arm2Shooting){
            arm2CD.reset();
        }else{
            TMa2 = true;
            if(arm2CD.seconds() > shootDur + stuckStartDur){
                arm2.setPosition(0);
                arm2Shooting = false;
                shooting = false;

                arm1Stuck = false;
                arm2Stuck = false;
                arm3Stuck = false;
                stuckStartCD.reset();

            }else{
                if(stuckStartCD.seconds() > stuckStartDur)arm2.setPosition(kickRot);
            }

        }



        if(!arm3Shooting){
            arm3CD.reset();
        }else{
            TMa3 = true;
            if(arm3CD.seconds() > shootDur + stuckStartDur){
                arm3.setPosition(0);
                arm3Shooting = false;
                shooting = false;

                arm1Stuck = false;
                arm2Stuck = false;
                arm3Stuck = false;
                stuckStartCD.reset();

            }else{
                if(stuckStartCD.seconds() > stuckStartDur)arm3.setPosition(kickRot);
            }

        }
    }



    float sensor1hue;
    float sensor2hue;
    float sensor3hue;

    float sensorGain = 55f;

    float purpleHMinV3 = 170;


    float greenSMinV3 = 0.55f;
    public void updateColors() {
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




        telem.addLine("=== Sensor 1 =========");
        telem.addData("H", sensor1hue);
        telem.addData("S", JavaUtil.colorToSaturation(sensor1Colors.toColor()));
        telem.addData("V", JavaUtil.colorToValue(sensor1Colors.toColor()));

        //telemetry.addData("dist", sensor1.getDistance(DistanceUnit.MM));
        telem.addData("Pos1", pos1);

        telem.addLine("=== Sensor 2 =========");
        telem.addData("H", sensor2hue);
        telem.addData("S", JavaUtil.colorToSaturation(sensor2Colors.toColor()));
        telem.addData("V", JavaUtil.colorToValue(sensor2Colors.toColor()));
        //telemetry.addData("dist", sensor2.getDistance(DistanceUnit.MM));
        telem.addData("Pos2", pos2);


        telem.addLine("=== Sensor 3 =========");
        telem.addData("H", sensor3hue);
        telem.addData("S", JavaUtil.colorToSaturation(sensor3Colors.toColor()));
        telem.addData("V", JavaUtil.colorToValue(sensor3Colors.toColor()));
        //telemetry.addData("dist", sensor3.getDistance(DistanceUnit.MM));
        telem.addData("Pos3", pos3);





    }

}
