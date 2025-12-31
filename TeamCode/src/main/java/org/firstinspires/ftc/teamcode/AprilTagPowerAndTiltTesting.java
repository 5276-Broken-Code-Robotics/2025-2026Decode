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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp



public class AprilTagPowerAndTiltTesting extends OpMode {



    private Servo pan;
    private Servo tilt;

    float power = 0.75f;


    double tiltangle = 0f;

    //maxes out at 0.59
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




    AprilTagDetection currOne;
    int state = -1;

    //0 : looking for
    //1 : orienting correctly

    boolean waiting = false;


    //0.75 for close sec 2

    boolean seen = false;

    CRServo transfer;
    ElapsedTime timer;



    Follower follower;
    double positionnecessary = 0f;

    AprilTagWebcam aprilTagWebCam = new AprilTagWebcam();
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setPose(new Pose(0,144));
        aprilTagWebCam.init(hardwareMap, telemetry);
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


        follower.setPose(new Pose(144,90));

        state=  -1;
    }








    public void loop(){

        telemetry.addData("Power : ", power);

        telemetry.addData("Tilt : ", tiltangle);

        if(gamepad1.aWasPressed()){
            power+=0.05f;
        }
        if(gamepad1.bWasPressed()){
            power-=0.05f;
        }
        if(power > 1){
            power = 1;
        }
        if(power < 0) power = 0;

        if(gamepad1.right_stick_y > 0 && state == -1){
            state = 0;

        }


        if(gamepad1.rightBumperWasPressed()){
            tiltangle+=0.05f;
        }
        if(gamepad1.leftBumperWasPressed()){
            tiltangle-=0.05f;
        }


        if(tiltangle > 0.59f){
            tiltangle = 0.59f;
        }
        if(tiltangle < 0) tiltangle = 0;







        telemetry.addData("curr y : ", follower.getPose().getY());

        intake.setPower(1);
        if(state <= 2)transfer.setPower(-1);



        telemetry.addData("Power : ",  power);


        if(aprilTagWebCam.getDetectedTags() != null){
            telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());
        }else{
            telemetry.addData("Save ", "Me");
        }


        if(state == 0) {


            if (elapsedTime.seconds() <= 1f) {
                waiting = false;
            } else if (elapsedTime.seconds() >= 1f && elapsedTime.seconds() <= 2f) {
                waiting = true;
            } else if (elapsedTime.seconds() >= 2f) {
                waiting = false;
                elapsedTime.reset();

                initpos = pan.getPosition();
            }


            if (aprilTagWebCam.getDetectedTags().isEmpty()) {

                if (!waiting) pan.setPosition(initpos + 0.05f);

            } else {

                seen = true;

                telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());

                telemetry.addData("We did it", "john");
                state = 1;

                currOne = aprilTagWebCam.getDetectedTags().get(0);
            }


        }

        if(state == 1){


            positionnecessary = pan.getPosition() + currOne.ftcPose.bearing * 0.4/180;

            initposforseeingpreorient = (float)pan.getPosition();
            telemetry.addData("position going to" ,  pan.getPosition() + currOne.ftcPose.bearing * 0.4/180);

            //tilt.setPosition(tilt.getPosition() + currOne.ftcPose.elevation);
            state = 2;
        }

        if(state == 2){

            telemetry.addData("curpos" , pan.getPosition());

            telemetry.addData("position going to" ,  positionnecessary);

            telemetry.addData("initpos" ,  initposforseeingpreorient);

            pan.setPosition(positionnecessary);


            telemetry.addData("bearing is this : ", currOne.ftcPose.bearing);


            if(pan.getPosition() == positionnecessary){
                state = 3;

                transfer.setPower(1);
                flywheel.setPower(power);
            }


            aprilTagWebCam.displayDetectionTelemetry(currOne);


        }



        tilt.setPosition(tiltangle);

        telemetry.addData("state : ", state);


        if(state == 3){

            transfer.setPower(1);

            aprilTagWebCam.displayDetectionTelemetry(currOne);
            if(elapsedTime.seconds() > 3){
                state = -1;
            }

        }
        follower.update();

        aprilTagWebCam.update();



    }


}
