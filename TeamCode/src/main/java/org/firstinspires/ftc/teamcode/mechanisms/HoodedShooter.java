package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.*;
import org.firstinspires.ftc.teamcode.mechanisms.ShootConstants;
public class HoodedShooter {



    private Servo pan;
    private Servo tilt;

    AprilTagWebcam aprilTagWebCam;

    float initposforseeingpreorient = 0f;
    boolean beganshot = false;

    double initpos = 0f;

    double positionnecessary;

    String state;

    ElapsedTime elapsedTime;
    private DcMotor flywheel;

    private DcMotor intake;

    AprilTagDetection aprilTag;

    boolean waiting = false;


    //0.75 for close sec 2

    boolean seen = false;

    double flywheelPower;

    CRServo transfer;

    boolean shotbegan = false;

    // (160 (vbig gear) / 18 (small gear)  * 20 (degrees of rotation))/300
    double maxTilt = 0.59;
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTagWebCam = new AprilTagWebcam();
        aprilTagWebCam.init(hardwareMap, telemetry);
        tilt = hardwareMap.get(Servo.class, "tilt");
        pan = hardwareMap.get(Servo.class, "pan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intake.setDirection(DcMotor.Direction.REVERSE);

        elapsedTime = new ElapsedTime();
    }

    public void loop()
    {
        if(shotbegan)OrientAndShoot();
    }

    public void BeginShot(){
        elapsedTime.reset();
        shotbegan = true;
    }


    public void OrientAndShoot(){
        state = "looking_for_april_tag";

        if(aprilTagWebCam.getDetectedTags() != null){
            telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());
        }

        telemetry.addData("Seen val :" , seen);


        if(state.equals("looking_for_april_tag")) {
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

                state = "found_tag_orienting";

                aprilTag = aprilTagWebCam.getDetectedTags().get(0);
            }


        }

        if(state.equals("found_tag_orienting")){
            double distance = 0;

            flywheelPower = ShootConstants.powerFromDistance(distance);

            //tilt.setPosition();

            positionnecessary = pan.getPosition() + aprilTag.ftcPose.bearing * 0.4/180;

            initposforseeingpreorient = (float)pan.getPosition();
            telemetry.addData("position going to" ,  pan.getPosition() + aprilTag.ftcPose.bearing * 0.4/180);

            //tilt.setPosition(tilt.getPosition() + currOne.ftcPose.elevation);
            state = "shoot";
        }

        if(state.equals("shoot")){

            pan.setPosition(positionnecessary);


            telemetry.addData("bearing is this : ", aprilTag.ftcPose.bearing);


            if(pan.getPosition() == positionnecessary){
                state = "cleanup_shoot";

                flywheel.setPower(flywheelPower);
                elapsedTime.reset();

            }

            aprilTagWebCam.displayDetectionTelemetry(aprilTag);

        }



        telemetry.addData("state : ", state);


        if(state.equals("cleanup_shoot")){
            if(elapsedTime.seconds() >= 1){
                transfer.setPower(1);
                aprilTagWebCam.displayDetectionTelemetry(aprilTag);
            }

            if(elapsedTime.seconds() >= 2.5 + 1){
                beganshot = false;
                transfer.setPower(-1);
            }
        }


        aprilTagWebCam.update();


    }
}
