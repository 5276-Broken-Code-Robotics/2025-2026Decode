package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Math.PI;


import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.*;

public class HoodedShooter {

    UseFreeSortHSV freeSort;


    private DcMotor pan;
    private Servo tilt;

    ElapsedTime shootTimer;

    double initpos = 0f;

    double positionnecessary;

    String state;

    ElapsedTime elapsedTime;


    private DcMotor flywheel1;

    private DcMotor flywheel2;



    //0.75 for close sec 2

    double flywheelPower;


    Telemetry telemetry;

    public boolean shotbegan = false;

    boolean resetting = false;

    ElapsedTime waitrotate;

    double angleToAprilTag = 0;
    Pose aprilTagRed = new Pose(144,144);
    Pose aprilTagBlue = new Pose(0,144);
    boolean rotated = false;
    Pose currentAprilTagPos = new Pose(0,0,0);


    int lastOrientedPos;
    boolean isAutoShot;

    Pose initPose;
    int currentID;
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    // (160 (vbig gear) / 18 (small gear)  * 20 (degrees of rotation))/300
    double maxTilt = 0.59;
    public void init(HardwareMap hardwareMap, Telemetry telemetry, GoBildaPinpointDriver pinpoint) {

        shootTimer = new ElapsedTime();

        freeSort = new UseFreeSortHSV();

        freeSort.init(hardwareMap, telemetry);

        shootTimer.reset();



        tilt = hardwareMap.get(Servo.class, "tilt");
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel1");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");


        pan = hardwareMap.get(DcMotor.class,"rot");
        pan.setTargetPosition(0);
        pan.setPower(1);
        pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        this.pinpoint = pinpoint;


        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitrotate = new ElapsedTime();
        resetting = false;


        limelight.start();

        this.telemetry = telemetry;




        elapsedTime = new ElapsedTime();
    }

    public void loop()
    {
        freeSort.loop();
        /*
        telemetry.addData("Flywheel power", flywheel1.getPower() + " " + flywheel2.getPower());
        telemetry.addData("Pan position : ", pan.getCurrentPosition());



        telemetry.addData("Rotated : ", rotated);
        telemetry.addData("State : ", state);


        telemetry.addData("numchanges", numchanges);
        telemetry.addData("IsAutoShot : ", isAutoShot);
        telemetry.addData("Shot status : ", shotbegan);

        telemetry.addData("Num shots : ", numshots);

         */


        pinpoint.update();
        double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        limelight.updateRobotOrientation(robotHeading + pan.getCurrentPosition() / 537.7/2 * PI);



        LLResult Llresult = limelight.getLatestResult();

        if(Llresult!= null && Llresult.isValid()){

            Pose3D botpose = Llresult.getBotpose_MT2();


            /*
            telemetry.addData("PX : ",botpose.getPosition().x);

            telemetry.addData("PY :", botpose.getPosition().y);

            telemetry.addData("Orient : ", botpose.getOrientation());

             */

        }


        //Orienting();


        pan.setTargetPosition(0);
        flywheel1.setPower(0.6);
        flywheel2.setPower(0.6);
        tilt.setPosition(0.075);

        //telemetry.update();


    }

    public void AutoBeginShot(boolean isRed, boolean isFar){

        isAutoShot = true;
        flywheelPower = 0.63; // Needs testing for accurate value
        tilt.setPosition(0); // Needs testing for accurate value
        elapsedTime.reset();



        waitrotate.reset();

        if(isRed &&!isFar){
            positionnecessary = 0.29/0.45 * 286/(0.45);
            flywheelPower = 0.63; // Needs testing for accurate value
            tilt.setPosition(0); // Needs testing for accurate value
        }

        if(!isRed && !isFar){
            positionnecessary = 0.11/0.45* 286/(0.45);
            flywheelPower = 0.63; // Needs testing for accurate value
            tilt.setPosition(0); // Needs testing for accurate value
        }

        if(isRed && isFar){
            positionnecessary = 0.38* 286/(0.45);
            flywheelPower = 0.8; // Needs testing for accurate value
            tilt.setPosition(0.275); // Needs testing for accurate value
        }

        if(!isRed && isFar){
            positionnecessary = (0.45-0.38)* 286/(0.45);
            flywheelPower = 0.8; // Needs testing for accurate value
            tilt.setPosition(0.275); // Needs testing for accurate value
        }

        pan.setTargetPosition((int)positionnecessary);
        shotbegan = true;



        shotbegan = false;
    }



    public void BeginShot(int id){


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



    boolean firingAllThree = false;
    public void fireThree(){
        firingAllThree = true;
    }


    public void FiringAllThree(){
        if(firingAllThree){
            //implement
        }
    }

    public void Orienting(){

        if(Math.abs(angleToAprilTag) <= PI/2 + 0.1 && Math.abs(lastOrientedPos - initpos) > 10){
            angleToAprilTag = Math.atan2((currentAprilTagPos.getY() - pinpoint.getPosY(DistanceUnit.INCH)),(currentAprilTagPos.getX() - pinpoint.getPosX(DistanceUnit.INCH)));
            initpos = turnAngle(pinpoint.getHeading(AngleUnit.RADIANS),angleToAprilTag) * (537/2)/(Math.PI);
            pan.setTargetPosition((int)initpos);
            lastOrientedPos = (int)initpos;
        }


    }



    public void Fire(int num){
        if(!freeSort.shooting)freeSort.shoot(num);
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


