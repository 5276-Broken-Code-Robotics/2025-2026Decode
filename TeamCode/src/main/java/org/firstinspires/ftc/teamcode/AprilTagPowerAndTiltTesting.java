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

    float power = 0.63f;


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


    boolean resetting = false;
    CRServo transfer;

    float intakepower = 1;
    ElapsedTime timer;



    Follower follower;
    double positionnecessary = 0f;


    int scanstate = -1;

    boolean locked = true;
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

        scanstate = -1;
    }








    public void loop(){


        telemetry.addData("Controls : ", "Square to increase power, circle to decrease. Triangle to fire. Right and left bumpers to increase and decrease tilt angles. Triangle to fire. X to stop." );
        telemetry.addData("Power : ", power);

        telemetry.addData("Tilt : ", tiltangle);



        telemetry.addData("intake power : ", intakepower );



        telemetry.addData("Is empty ? : ", aprilTagWebCam.getDetectedTags().isEmpty());
        if(gamepad1.squareWasPressed()){
            power+=0.1f;
        }
        if(gamepad1.circleWasPressed()){
            power-=0.1f;
        }
        if(power > 1){
            power = 1;
        }

        if(pan.getPosition() > 0.4){
            pan.setPosition(0);
            elapsedTime.reset();
            resetting = true;
        }

        if(elapsedTime.seconds() > 2){
            resetting = false;
        }


        if(power < 0) power = 0;

        if(gamepad1.triangleWasPressed() && state == -1){
            state = 0;

        }


        if(gamepad1.rightBumperWasPressed()){
            tiltangle+=0.025f;
        }
        if(gamepad1.leftBumperWasPressed()){
            tiltangle-=0.025f;
        }


        
        
        if(gamepad1.right_stick_y > 0 && scanstate == -1){

            scanstate = 0;

            telemetry.addData("Scanning", " Yes i am scanning");

        }



        telemetry.addData("Power : ", flywheel.getPower());
        telemetry.addData("Scan state : ", scanstate);

        if(scanstate == 0 && ! resetting){




            if (elapsedTime.seconds() <= 1) {
                waiting = false;
            } else if (elapsedTime.seconds() >= 1 && elapsedTime.seconds() <= 2) {
                waiting = true;
            } else if (elapsedTime.seconds() >= 2) {
                waiting = false;
                elapsedTime.reset();

                initpos = pan.getPosition();
            }


            if (aprilTagWebCam.getDetectedTags().isEmpty()) {

                if (!waiting) pan.setPosition(initpos + 0.1);




                telemetry.addData("We are empty", "for some reason");

            } else {



                seen = true;

                telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());

                telemetry.addData("We did it", "john");
                scanstate = 1;


                pan.setPosition(pan.getPosition());
                currOne = aprilTagWebCam.getDetectedTags().get(0);
            }


        }
        if(scanstate == 1){

            if(Math.abs(currOne.ftcPose.bearing) > 5){

                positionnecessary = pan.getPosition() + currOne.ftcPose.bearing * 0.4/180;


                telemetry.addData("Bearing : ", currOne.ftcPose.bearing);
                initposforseeingpreorient = (float)pan.getPosition();

                telemetry.addData("Current position :", pan.getPosition());
                telemetry.addData("position going to" ,  pan.getPosition() + currOne.ftcPose.bearing * 0.4/180);


                telemetry.update();
                pan.setPosition(positionnecessary);
                scanstate = -1;

            }else{
                scanstate = -1;

            }


        }



        if(tiltangle > 0.59f){
            tiltangle = 0.59f;
        }
        if(tiltangle < 0) tiltangle = 0;



        if(gamepad1.dpadLeftWasPressed()){
            pan.setPosition(pan.getPosition() -  0.1);
        }

        if(gamepad1.dpadRightWasPressed()){
            pan.setPosition(pan.getPosition() + 0.1);
        }



        if(gamepad1.dpadUpWasPressed()){
            intakepower+=0.1;
        }


        if(gamepad1.dpadDownWasPressed()){
            intakepower-=0.1f;
        }






        telemetry.addData("curr y : ", follower.getPose().getY());

        intake.setPower(intakepower);
        if(state <= 2)transfer.setPower(-1);





        if(aprilTagWebCam.getTagByID(24)!=null)telemetry.addData("Distance : " , aprilTagWebCam.getTagByID(24).ftcPose.y);

        telemetry.addData("Power : ",  power);


        if(aprilTagWebCam.getDetectedTags() != null){
            telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());
        }else{
            telemetry.addData("Save ", "Me");
        }


        if(state == 0){

            state = 3;
            flywheel.setPower(power);
            elapsedTime.reset();



        }



        tilt.setPosition(tiltangle);

        telemetry.addData("state : ", state);


        if(state == 3){
            flywheel.setPower(power);


            if(elapsedTime.seconds() > 3){
                transfer.setPower(1);
            }

            //aprilTagWebCam.displayDetectionTelemetry(currOne);
            if(gamepad1.xWasPressed()){

                flywheel.setPower(0);
                state = -1;
            }

        }
        follower.update();

        aprilTagWebCam.update();



    }


}
