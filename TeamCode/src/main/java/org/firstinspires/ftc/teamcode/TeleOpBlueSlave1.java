package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp



public class TeleOpBlueSlave1 extends OpMode {
    private Servo pan;
    private Servo tilt;

    float power = 0.75f;
    double tiltangle = 0.3f;




    float turretangle = 0f;

    float initposforseeingpreorient = 0f;
    boolean beganshot = false;

    double initpos = 0f;

    private DcMotor fr;

    private DcMotor fl;

    private DcMotor br;
    private DcMotor bl;




    FieldRelativeDrive fieldDrive;
    ElapsedTime elapsedTime;
    private DcMotor flywheel;

    private DcMotor intake;


    Follower follower;



    int state = 0;

    //0 : looking for
    //1 : orienting correctly

    boolean waiting = false;


    //0.75 for close sec 2

    boolean seen = false;

    CRServo transfer;
    ElapsedTime timer;



    //Follower follower;
    double positionnecessary = 0f;




    Pose aprilTagRed = new Pose(144,144);
    Pose aprilTagBlue = new Pose(0,144);

    HoodedShooter hShooter;


    int num = 0;
    public void init() {

        num = 0;

//        follower = Constants.createFollower(hardwareMap);
//
//        follower.setPose(new Pose(0,144));




        follower = Constants.createFollower(hardwareMap);


         // Lowest (Third Set) of Artifacts from the Spike Mark.


        follower.setStartingPose(new Pose(144-96, 24, Math.toRadians(180)));


        follower.setPose(new Pose(144-96, 24, Math.toRadians(180)));

        follower.update();
        timer = new ElapsedTime();
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        tilt = hardwareMap.get(Servo.class, "tilt");
        pan = hardwareMap.get(Servo.class, "pan");




        pan.setPosition(0);

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intake.setDirection(DcMotor.Direction.REVERSE);

        timer.reset();
        beganshot = false;
        elapsedTime = new ElapsedTime();

        elapsedTime.reset();

        hShooter = new HoodedShooter();




        hShooter.init(hardwareMap,telemetry, follower, fl, fr, bl, br);

        state=  0;
    }

    public void loop(){

        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);





        if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0)drive(-gamepad2.left_stick_y/2, gamepad2.left_stick_x/2, gamepad2.right_stick_x/2);


        if(gamepad2.leftBumperWasPressed()){
            intake.setPower(0.25);
        }

        if(gamepad2.rightBumperWasPressed()){
            intake.setPower(1);
        }

        if(gamepad1.left_bumper && num == 0){
            follower.update();
            hShooter.AutoBeginShot(true);

            pan.setPosition(0.29);
            num = 1;
            elapsedTime.reset();
        }


        telemetry.addData("ms", elapsedTime.milliseconds());




        telemetry.addData("Num : " , num);


        if(gamepad1.right_bumper && num == 0){

            follower.update();
            hShooter.BeginShot(20);


            num = 1;
            elapsedTime.reset();


        }

        if(gamepad1.left_bumper && num == 0){
            follower.update();
            hShooter.AutoBeginShot(false);

            pan.setPosition(0.11);
            num = 1;
            elapsedTime.reset();
        }


        if(elapsedTime.seconds() > 2){
            num = 0;
        }


        transfer.setPower(-1);


        hShooter.loop();

        if(num == 1){
            telemetry.addData("We", "Are being run2");
        }

        follower.update();
    }


    public void drive(double forward, double strafe, double rotate) {
        double flPower = forward + strafe + rotate;
        double frPower = forward - strafe - rotate;
        double blPower = forward - strafe + rotate;
        double brPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));

        fl.setPower((maxSpeed * (flPower / maxPower)));
        fr.setPower((maxSpeed * (frPower / maxPower)));
        bl.setPower((maxSpeed * (blPower / maxPower)));
        br.setPower((maxSpeed * (brPower / maxPower)));
    }


}
