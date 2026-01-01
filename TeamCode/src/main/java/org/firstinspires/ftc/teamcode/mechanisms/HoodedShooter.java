package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.*;

public class HoodedShooter {



    private Servo pan;
    private Servo tilt;

    AprilTagWebcam aprilTagWebCam;

    float initposforseeingpreorient = 0f;

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

    Telemetry telemetry;

    public boolean shotbegan = false;

    Follower follower;


    double angleToAprilTag = 0;
    Pose aprilTagRed = new Pose(129.61653272101034,128.62456946039035);
    Pose aprilTagBlue = new Pose(15.2,128.62456946039035);
    private PathChain rotate1;

    boolean rotated = false;
    Pose currentAprilTagPos;

    boolean isAutoShot;

    Pose initPose;
    int currentID;
    float tiltangle = 0f;


    private DcMotor fr;

    private DcMotor fl;

    private DcMotor br;
    private DcMotor bl;
    // (160 (vbig gear) / 18 (small gear)  * 20 (degrees of rotation))/300
    double maxTilt = 0.59;
    public void init(HardwareMap hardwareMap, Follower follower) {
        aprilTagWebCam = new AprilTagWebcam();
        aprilTagWebCam.init(hardwareMap, telemetry);
        tilt = hardwareMap.get(Servo.class, "tilt");
        pan = hardwareMap.get(Servo.class, "pan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");;
        bl = hardwareMap.get(DcMotor.class, "bl");;
        br = hardwareMap.get(DcMotor.class, "br");;

        this.follower = follower;

        elapsedTime = new ElapsedTime();
    }

    public void loop()
    {
        telemetry.addData("Pan position : ", pan.getPosition());


        telemetry.addData("Rotated : ", rotated);

        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addData("Shot status : ", shotbegan);
        if(shotbegan){

            OrientAndShoot();


            double angleDiff = 180/Math.PI * turnAngle(follower.getPose().getHeading(), Math.atan2((currentAprilTagPos.getY() - follower.getPose().getY()),(currentAprilTagPos.getX() - follower.getPose().getX())));
            telemetry.addData("Angle difference in Degrees : ", angleDiff);
            telemetry.addData("Position : ", (int)follower.getPose().getX() + " " + (int)follower.getPose().getY());
            telemetry.update();

        }else{
            telemetry.addData("we are not shooting", "i cry");
        }
    }

    public void AutoBeginShot(){
        BeginShot(24);
        isAutoShot = true;

        flywheelPower = 0.65; // Needs testing for accurate value
        tilt.setPosition(0.5); // Needs testing for accurate value
    }

    public void BeginShot(int id){
        elapsedTime.reset();
        state = "chassis_orient_to_tag";
        currentID = id;
        if(id == 24){
            currentAprilTagPos = aprilTagRed;
        }


        initPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        rotated = false;
        shotbegan = true;
        initpos = 0;


    }


    public void OrientAndShoot(){

        if(aprilTagWebCam.getDetectedTags() != null){
            //telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());
        }



        if(state.equals("chassis_orient_to_tag")){
            angleToAprilTag = Math.atan2((currentAprilTagPos.getY() - follower.getPose().getY()),(currentAprilTagPos.getX() - follower.getPose().getX()));



            telemetry.addData("Turning angle" ,turnAngle(follower.getPose().getHeading(),angleToAprilTag));
            telemetry.addData("Angle to aprilTag" , angleToAprilTag);


            if(Math.abs(turnAngle(follower.getPose().getHeading(),angleToAprilTag)) > Math.PI/4) {




                if(turnAngle(follower.getPose().getHeading(),angleToAprilTag) > 0){
                    fr.setPower(0.8);
                    fl.setPower(-0.8);
                    bl.setPower(-0.8);
                    br.setPower(0.8);
                }else{
                    fr.setPower(-0.8);
                    fl.setPower(0.8);
                    bl.setPower(0.8);
                    br.setPower(-0.8);
                }


                //Maybe tweak to make it rotate just to the point that it needs to rotate to the edge of the AprilTag Range

            }
            else{
                state = "looking_for_april_tag";

                fl.setPower(0);
                fr.setPower(0);

                br.setPower(0);

                bl.setPower(0);
                if(turnAngle(follower.getPose().getHeading(),angleToAprilTag) > 0){
                    pan.setPosition(0.4 * turnAngle(follower.getPose().getHeading(),angleToAprilTag) / 180);

                }else{
                    pan.setPosition(0.4 - 0.4 * turnAngle(follower.getPose().getHeading(),angleToAprilTag) / 180);
                }
            }
        }



        if(state.equals("looking_for_april_tag")) {

            if (elapsedTime.seconds() <= ShootConstants.aprilTagMoveScanTimePerStep_seconds) {
                waiting = false;
            } else if (elapsedTime.seconds() >= ShootConstants.aprilTagMoveScanTimePerStep_seconds && elapsedTime.seconds() <= ShootConstants.aprilTagMoveScanTimePerStep_seconds + ShootConstants.aprilTagScanTimePerStep_seconds) {
                waiting = true;
            } else if (elapsedTime.seconds() > ShootConstants.aprilTagMoveScanTimePerStep_seconds + ShootConstants.aprilTagScanTimePerStep_seconds) {
                waiting = false;
                elapsedTime.reset();

                initpos = pan.getPosition();
            }


            if (aprilTagWebCam.getDetectedTags().isEmpty()) {

                if (!waiting){

                    if(initpos > 0.4){
                        initpos = 0;
                    }
                    pan.setPosition(initpos + 0.05f);
                }

            } else {
                boolean found = false;

                //telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());

                for(int i =0; i < aprilTagWebCam.getDetectedTags().size();i++){
                    if(aprilTagWebCam.getDetectedTags().get(i).id == currentID){

                        state = "found_tag_orienting";
                        aprilTag = aprilTagWebCam.getDetectedTags().get(i);
                        found = true;
                    }
                }

                if(!found){
                    if (!waiting) pan.setPosition(initpos + 0.05f);
                }
            }


        }

        if(state.equals("found_tag_orienting")){

            double distance = aprilTag.ftcPose.y;

            if(!isAutoShot) {
                flywheelPower = ShootConstants.powerFromDistance(distance);
                tilt.setPosition(ShootConstants.tiltFromDistance(distance));
            }

            positionnecessary = pan.getPosition() + aprilTag.ftcPose.bearing * 0.4/180;

            initposforseeingpreorient = (float)pan.getPosition();
            //telemetry.addData("position going to" ,  pan.getPosition() + aprilTag.ftcPose.bearing * 0.4/180);

            //tilt.setPosition(tilt.getPosition() + currOne.ftcPose.elevation);
            state = "shoot";
        }

        if(state.equals("shoot")){

            pan.setPosition(positionnecessary);


            //telemetry.addData("bearing is this : ", aprilTag.ftcPose.bearing);


            if(Math.abs(pan.getPosition() - positionnecessary) < 0.01){
                state = "cleanup_shoot";

                flywheel.setPower(flywheelPower);
                elapsedTime.reset();

            }

            aprilTagWebCam.displayDetectionTelemetry(aprilTag);

        }



        telemetry.addData("state : ", state);


        if(state.equals("cleanup_shoot")){
            if(elapsedTime.seconds() >= ShootConstants.flywheelAccelerationTime_seconds){
                transfer.setPower(1);
                aprilTagWebCam.displayDetectionTelemetry(aprilTag);
            }

            if(elapsedTime.seconds() >= ShootConstants.shotDuration_seconds + ShootConstants.flywheelAccelerationTime_seconds){
                shotbegan = false;
                isAutoShot = false;

                //flywheel.setPower(0); probably better to keep flywheel running
                transfer.setPower(-1);

            }
        }


        aprilTagWebCam.update();


        follower.update();


    }

    public static double turnAngle(double currentHeading, double targetAngle) {
        double error = targetAngle - currentHeading;

        // Normalize to (-PI, PI]
        while (error > Math.PI)  error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;

        return error;
    }
}
