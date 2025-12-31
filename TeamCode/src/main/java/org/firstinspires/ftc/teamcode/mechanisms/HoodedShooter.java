package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
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

    boolean shotbegan = false;

    Follower follower;


    double angleToAprilTag = 0;
    Pose aprilTagRed = new Pose(129.61653272101034,128.62456946039035);
    Pose aprilTagBlue = new Pose(15.2,128.62456946039035);
    private PathChain rotate1;

    boolean rotated = false;
    Pose currentAprilTagPos;

    Pose initPose;
    int currentID;
    float tiltangle = 0f;


    private DcMotor fr;

    private DcMotor fl;

    private DcMotor br;
    private DcMotor bl;
    // (160 (vbig gear) / 18 (small gear)  * 20 (degrees of rotation))/300
    double maxTilt = 0.59;
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        aprilTagWebCam = new AprilTagWebcam();
        aprilTagWebCam.init(hardwareMap, telemetry);
        tilt = hardwareMap.get(Servo.class, "tilt");
        pan = hardwareMap.get(Servo.class, "pan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intake.setDirection(DcMotor.Direction.REVERSE);


        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;






        this.follower = follower;

        this.telemetry = telemetry;


        elapsedTime = new ElapsedTime();
    }



    public void loop()
    {


        telemetry.addData("Rotated : ", rotated);

        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addData("Shot status : ", shotbegan);
        if(shotbegan){

            OrientAndShoot();
        }else{
            telemetry.addData("we are not shooting", "i cry");
        }

        telemetry.update();



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
        follower.update();


        if(aprilTagWebCam.getDetectedTags() != null){
            //telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());
        }



        if(state.equals("chassis_orient_to_tag")){
            angleToAprilTag = Math.atan2((aprilTagRed.getY() - follower.getPose().getY()),(aprilTagRed.getX() - follower.getPose().getX()));



            telemetry.addData("Turning angle" ,turnAngle(follower.getPose().getHeading(),angleToAprilTag));
            telemetry.addData("Angle to aprilTag" , angleToAprilTag);


            if(Math.abs(turnAngle(follower.getPose().getHeading(),angleToAprilTag)) > Math.PI/2) {




                if(turnAngle(follower.getPose().getHeading(),angleToAprilTag) > 0){
                    fr.setPower(0.7);
                    fl.setPower(-0.7);
                    bl.setPower(-0.7);
                    br.setPower(0.7);
                }else{
                    fr.setPower(-0.7);
                    fl.setPower(0.7);
                    bl.setPower(0.7);
                    br.setPower(-0.7);
                }


                //Maybe tweak to make it rotate just to the point that it needs to rotate to the edge of the AprilTag Range
                /*
                if (!rotated) {
                    follower.turnTo(angleToAprilTag);
                    elapsedTime.reset();
                    rotated = true;
                }


                if (!follower.isBusy() || elapsedTime.seconds() > 1.5) {
                    rotated = false;
                    state = "looking_for_april_tag";
                }
                 */
            }
            else{
                state = "looking_for_april_tag";

                fl.setPower(0);
                fr.setPower(0);

                br.setPower(0);

                bl.setPower(0);

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

                if (!waiting) pan.setPosition(initpos + 0.05f);

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


            flywheelPower = ShootConstants.powerFromDistance(distance);

            tilt.setPosition(ShootConstants.tiltFromDistance(distance));

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

                flywheel.setPower(0);
                transfer.setPower(-1);
            }
        }


        aprilTagWebCam.update();



        telemetry.update();
    }

    public static double turnAngle(double currentHeading, double targetAngle) {
        double error = targetAngle - currentHeading;

        // Normalize to (-PI, PI]
        while (error > Math.PI)  error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;

        return error;
    }
}
