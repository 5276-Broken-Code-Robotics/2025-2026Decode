package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp



public class HoodedShooterTest extends OpMode {



    private Servo pan;
    private Servo tilt;

    float power = 0.75f;





    double tiltangle = 0.3f;
    private DcMotor fr;




    float turretangle = 0f;

    float initposforseeingpreorient = 0f;
    boolean beganshot = false;

    double initpos = 0f;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;



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




    Pose aprilTagRed = new Pose(129.61653272101034,128.62456946039035);
    Pose aprilTagBlue = new Pose(15.2,128.62456946039035);

    HoodedShooter hShooter;


    int num = 0;
    public void init() {

        num = 0;





//
//        follower = Constants.createFollower(hardwareMap);
//
//        follower.setPose(new Pose(0,144));




        follower = Constants.createFollower(hardwareMap);

        follower.setPose(new Pose(0,0, Math.PI));
        timer = new ElapsedTime();
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        tilt = hardwareMap.get(Servo.class, "tilt");
        pan = hardwareMap.get(Servo.class, "pan");




        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        intake.setDirection(DcMotor.Direction.REVERSE);

        timer.reset();
        beganshot = false;
        elapsedTime = new ElapsedTime();

        elapsedTime.reset();

        hShooter = new HoodedShooter();

        hShooter.init(hardwareMap,telemetry, follower);


        state=  0;
    }









    public void loop(){

        telemetry.addData("ms", elapsedTime.milliseconds());
        intake.setPower(1);

        telemetry.addData("Num : " , num);





        if(gamepad1.right_bumper && num == 0){
            hShooter.BeginShot(24);
            num = 1;
            elapsedTime.reset();

        }

        if(elapsedTime.seconds() > 2){
            num = 0;
        }




        hShooter.loop();

        if(num == 1){
            telemetry.addData("We", "Are being run2");
        }
    }


}
