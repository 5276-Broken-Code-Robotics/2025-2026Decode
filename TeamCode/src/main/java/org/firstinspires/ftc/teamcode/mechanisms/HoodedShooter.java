package org.firstinspires.ftc.teamcode.mechanisms;


import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class HoodedShooter {

    FinalFreeSortHSV freeSort;


    private Servo tilt;

    ElapsedTime shootTimer;


    public char p1;
    public char p2;
    public char p3;

    double initpos = 0f;

    double positionnecessary;


    int id = 0;
    String state;

    ElapsedTime elapsedTime;





    private DcMotor flywheel1;

    private DcMotor flywheel2;



    //0.75 for close sec 2

    public double flywheelPower;

    public double headingTiltPos;
    Telemetry telemetry;

    public boolean shotbegan = false;

    boolean resetting = false;

    ElapsedTime waitrotate;

    double angleToAprilTag = 0;
    Pose2D aprilTagRed = new Pose2D(DistanceUnit.INCH,144,144, AngleUnit.RADIANS,0);
    Pose2D aprilTagBlue = new Pose2D(DistanceUnit.INCH,0,144, AngleUnit.RADIANS,0);
    boolean rotated = false;
    Pose2D currentAprilTagPos;


    int lastOrientedPos;
    boolean isAutoShot;

    Pose initPose;
    int currentID;
    Limelight3A limelight;

    float faceForwardPos = 105;

    float maxTurn = 525;
    GoBildaPinpointDriver pinpoint;

    public ElapsedTime pinpointUpdatePause;
    // (160 (vbig gear) / 18 (small gear)  * 20 (degrees of rotation))/300
    double maxTilt = 0.59;
    public void init(HardwareMap hardwareMap, Telemetry telemetry, GoBildaPinpointDriver pinpoint, int id) {

        shootTimer = new ElapsedTime();

        freeSort = new FinalFreeSortHSV();

        freeSort.init(hardwareMap, telemetry);

        shootTimer.reset();


        pinpointUpdatePause = new ElapsedTime();
        tilt = hardwareMap.get(Servo.class, "tilt");
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");


        this.id = id;


        this.pinpoint = pinpoint;


        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitrotate = new ElapsedTime();
        resetting = false;




        flywheelPower = 0.6;
        headingTiltPos = 0.12;
        panTracking = new PanTracking();


        if(id == 24){
            currentAprilTagPos = aprilTagRed;
        }else{
            currentAprilTagPos = aprilTagBlue;
        }


        currentID = id;


        panTracking.init(hardwareMap,pinpoint,telemetry,limelight,id,currentAprilTagPos);



        //limelight.start();
        //DISABLED FOR POWER PURPOSES, DO NOT FORGET TO REENABLE


        this.telemetry = telemetry;




        elapsedTime = new ElapsedTime();
    }

    public void start(){
        pinpointUpdatePause.reset();
        panTracking.start();
    }

    public void loop()
    {

        //Alter, this is just for red goal
        if((144-pinpoint.getPosY(DistanceUnit.INCH)) * (144-pinpoint.getPosY(DistanceUnit.INCH)) + (144-pinpoint.getPosX(DistanceUnit.INCH))*(144-pinpoint.getPosX(DistanceUnit.INCH)) < 3200 ){
            headingTiltPos = 0.051;
        }else if(pinpoint.getPosY(DistanceUnit.INCH) <= 36) {
            headingTiltPos = 0.075;
        }else{
            headingTiltPos = 0.1;
        }


        freeSort.loop();

        panTracking.loop();

        if(firingPat)FiringAPattern();

        p1 = freeSort.pos1;
        p2 = freeSort.pos2;
        p3 = freeSort.pos3;

        /*


        telemetry.addData("Flywheel power", flywheel1.getPower() + " " + flywheel2.getPower());
        telemetry.addData("Pan position : ", pan.getCurrentPosition());



        telemetry.addData("numchanges", numchanges);
        telemetry.addData("IsAutoShot : ", isAutoShot);
        telemetry.addData("Shot status : ", shotbegan);

        telemetry.addData("Num shots : ", numshots);

         */

        if(pinpointUpdatePause.seconds() > 0.3)pinpoint.update();



        //Orienting();




        flywheel1.setPower(flywheelPower);
        flywheel2.setPower(flywheelPower);
        tilt.setPosition(headingTiltPos);

        //telemetry.update();
    }

    public void AutoBeginShot(boolean isRed, boolean isFar){


        //ALL LEGACY VALUES GANG


        //ADJUST ON YOUR OWN


        isAutoShot = true;
        tilt.setPosition(0); // Needs testing for accurate value
        elapsedTime.reset();



        waitrotate.reset();

        if(isRed &&!isFar){
            positionnecessary = 0.29/0.45 * 286/(0.45);
            tilt.setPosition(0); // Needs testing for accurate value
        }

        if(!isRed && !isFar){
            positionnecessary = 0.11/0.45* 286/(0.45);
            tilt.setPosition(0); // Needs testing for accurate value
        }

        if(isRed && isFar){
            positionnecessary = 0.38* 286/(0.45);
            tilt.setPosition(0.275); // Needs testing for accurate value
        }

        if(!isRed && isFar){
            positionnecessary = (0.45-0.38)* 286/(0.45);
            tilt.setPosition(0.275); // Needs testing for accurate value
        }

    }



    public void BeginShot(){


        isAutoShot = false;
        elapsedTime.reset();
        state = "chassis_orient_to_tag";
        currentID = id;
        if(id == 24){
            currentAprilTagPos = aprilTagRed;
        }

        if(id == 20){
            currentAprilTagPos = aprilTagBlue;
        }

        rotated = false;

        shotbegan = true;


        shotbegan = false;
    }



    boolean firingPat = false;
    public void firePattern(char[] pat){

        if(pat.length > 0 && curpat.length > 0){
            firingPat = true;
            dex = 0;
            for(int i =0; i < pat.length;i++){
                curpat[i] = pat[i];
            }
        }

    }

    char[] curpat = {'p', 'p', 'p'};

    int dex = 0;

    public void FiringAPattern(){
            if(dex <= 3){
                if(!freeSort.shooting){
                    FireColor(curpat[dex]);
                    dex++;
                }
            }else{
                firingPat = false;
                dex = 0;
            }

    }


    PanTracking panTracking;



    public void Fire(int num){
        if(!freeSort.shooting)freeSort.shoot(num);
    }

    public void FireColor(char c){

        if(c == 'p'){
            if(freeSort.pos1 == 'p'){
                freeSort.shoot(1);
            }else
            if(freeSort.pos2 == 'p'){
                freeSort.shoot(2);
            }else
            if(freeSort.pos3 == 'p'){
                freeSort.shoot(3);
            }
        }else{
            if(freeSort.pos1 == 'g'){
                freeSort.shoot(1);
            }else
            if(freeSort.pos2 == 'g'){
                freeSort.shoot(2);
            }else
            if(freeSort.pos3 == 'g'){
                freeSort.shoot(3);
            }
        }



    }

    public static double turnAngle(double currentHeading, double targetAngle) {
        double error = targetAngle - currentHeading;

        // Normalize to (-PI, PI]
        while (error > Math.PI)  error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;

        return error;
    }

    public static double distTo(Pose tagPos, Pose ourPos){
        return Math.sqrt((ourPos.getY() - tagPos.getY())*(ourPos.getY() - tagPos.getY()) + (ourPos.getX() - tagPos.getX())*(ourPos.getX() - tagPos.getX()));
    }

    public static double AngleToTag(Pose tagPos, Pose ourPos){
        return Math.atan2(tagPos.getY() - ourPos.getY(), tagPos.getX() - ourPos.getX());
    }


}


