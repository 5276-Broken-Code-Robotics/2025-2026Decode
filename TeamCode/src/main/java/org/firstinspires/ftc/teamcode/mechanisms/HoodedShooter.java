package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.*;

public class HoodedShooter {

    ElapsedTime resetElapsedtime;



    private Servo pan;
    private Servo tilt;

    AprilTagWebcam aprilTagWebCam;





    boolean firing = false;
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

    public boolean isRed = false;

    double flywheelPower;

    CRServo transfer;

    Telemetry telemetry;

    public boolean shotbegan = false;

    Follower follower;

    ElapsedTime movementTrackCD;

    boolean resetting = false;

    ElapsedTime waitrotate;

    double angleToAprilTag = 0;
    Pose aprilTagRed = new Pose(144,144);
    Pose aprilTagBlue = new Pose(0,144);
    private PathChain rotate1;

    boolean rotated = false;
    Pose currentAprilTagPos = new Pose(0,0,0);

    boolean isAutoShot;

    Pose initPose;
    int currentID;
    float tiltangle = 0f;




    private DcMotor fr;

    private DcMotor fl;

    Pose previousPoseLastMovementTracked;

    private DcMotor br;
    private DcMotor bl;


    Pose posePreRotate;

    int numshots = 0;

    boolean moving = false;


    boolean waitedforautorotate = false;
    int numchanges = 0;
    // (160 (vbig gear) / 18 (small gear)  * 20 (degrees of rotation))/300
    double maxTilt = 0.59;
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {

        moving = false;
        aprilTagWebCam = new AprilTagWebcam();
        aprilTagWebCam.init(hardwareMap, telemetry);
        tilt = hardwareMap.get(Servo.class, "tilt");
        pan = hardwareMap.get(Servo.class, "pan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitrotate = new ElapsedTime();
        resetElapsedtime = new ElapsedTime();




        resetting = false;


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


        if(movementTrackCD.seconds() > 0.05){

            if(distTo(follower.getPose(), previousPoseLastMovementTracked) > 20){
                moving = true;
            }else{
                moving = false;
            }

            previousPoseLastMovementTracked = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());


            movementTrackCD.reset();
        }


        telemetry.addData("Flywheel power", flywheel.getPower());
        telemetry.addData("Pan position : ", pan.getPosition());


        telemetry.addData("Rotated : ", rotated);

        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addData("State : ", state);


        telemetry.addData("numchanges", numchanges);
        telemetry.addData("IsAutoShot : ", isAutoShot);
        telemetry.addData("Shot status : ", shotbegan);

        telemetry.addData("Num shots : ", numshots);


        if(shotbegan){

            OrientAndShoot();

            double angleDiff = 0;
            if(!isAutoShot) angleDiff = 180/Math.PI * turnAngle(follower.getPose().getHeading(), Math.atan2((currentAprilTagPos.getY() - follower.getPose().getY()),(currentAprilTagPos.getX() - follower.getPose().getX())));
            telemetry.addData("Angle difference in Degrees : ", angleDiff);
            //telemetry.addData("Position : ", (int)follower.getPose().getX() + " " + (int)follower.getPose().getY());



        }else{
            telemetry.addData("we are not shooting", "i cry");
        }



        telemetry.addData("Resetting", resetting);

        telemetry.addData("Resetting elapsed time : ", resetElapsedtime );



        telemetry.update();
    }

    public void AutoBeginShot(boolean isRed, boolean isFar){

        isAutoShot = true;




        flywheelPower = 0.63; // Needs testing for accurate value
        tilt.setPosition(0); // Needs testing for accurate value
        elapsedTime.reset();


        state = "shoot";

        waitrotate.reset();

        if(isRed &&!isFar){
            positionnecessary = 0.29;
            flywheelPower = 0.63; // Needs testing for accurate value
            tilt.setPosition(0); // Needs testing for accurate value
        }

        if(!isRed && !isFar){
            positionnecessary = 0.11;
            flywheelPower = 0.63; // Needs testing for accurate value
            tilt.setPosition(0); // Needs testing for accurate value
        }

        if(isRed && isFar){
            positionnecessary = 0.38;
            flywheelPower = 0.8; // Needs testing for accurate value
            tilt.setPosition(0.275); // Needs testing for accurate value
        }

        if(!isRed && isFar){
            positionnecessary = 0.45-0.38;
            flywheelPower = 0.8; // Needs testing for accurate value
            tilt.setPosition(0.275); // Needs testing for accurate value
        }


        pan.setPosition(positionnecessary);

        shotbegan = true;



    }

    public void BeginShot(int id){
        elapsedTime.reset();
        state = "chassis_orient_to_tag";
        currentID = id;
        if(id == 24){
            currentAprilTagPos = aprilTagRed;
        }

        if(id == 20){
            currentAprilTagPos = aprilTagBlue;
        }

        initPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        rotated = false;
        shotbegan = true;

        angleToAprilTag = Math.atan2((currentAprilTagPos.getY() - follower.getPose().getY()),(currentAprilTagPos.getX() - follower.getPose().getX()));




        initpos = 0.2;






    }


    boolean foundCorrectTag = false;
    public void Orienting(){

        if(isRed) {
            angleToAprilTag = Math.atan2(144-follower.getPose().getY(), 144-follower.getPose().getX());
        }else{
            angleToAprilTag = Math.atan2(144-follower.getPose().getY(), follower.getPose().getX());
        }

        initpos = turnAngle(follower.getPose().getHeading(),angleToAprilTag) * (0.45)/(Math.PI);
        pan.setPosition(initpos);





        for(int i =0; i < aprilTagWebCam.getDetectedTags().size();i++){
            if(aprilTagWebCam.getDetectedTags().get(i).id == currentID){
                aprilTag = aprilTagWebCam.getDetectedTags().get(i);
                Pose realPose = new Pose(currentAprilTagPos.getX() -( aprilTag.ftcPose.x * Math.cos(follower.getHeading()) -aprilTag.ftcPose.y * Math.sin(follower.getHeading())),currentAprilTagPos.getY() - ( aprilTag.ftcPose.x * Math.sin(follower.getHeading())  + aprilTag.ftcPose.y * Math.cos(follower.getHeading())));
                follower.setPose(realPose);
                foundCorrectTag = true;
            }
        }

    }


    public void OrientAndShoot(){


        //Skips stage


        if(state.equals("chassis_orient_to_tag")){
            follower.setPose(new Pose(initPose.getX(), initPose.getY(), follower.getPose().getHeading()));
            state = "looking_for_april_tag";


            resetting = true;
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
            bl.setPower(0);


            initpos = turnAngle(follower.getPose().getHeading(),angleToAprilTag) * (0.45)/(Math.PI);


            resetElapsedtime.reset();
        }


        angleToAprilTag = Math.atan2((currentAprilTagPos.getY() - follower.getPose().getY()),(currentAprilTagPos.getX() - follower.getPose().getX()));

        if(moving){
            initpos = turnAngle(follower.getPose().getHeading(),angleToAprilTag) * (0.45)/(Math.PI);
        }

        if(Math.abs(turnAngle(follower.getPose().getHeading(),angleToAprilTag)) > Math.PI/6) {
            state = "chassis_orient_to_tag";
        }



        transfer.setPower(-1);
        if(aprilTagWebCam.getDetectedTags() != null){
            //telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());
        }



        if(state.equals("chassis_orient_to_tag") && !moving){




            angleToAprilTag = Math.atan2((currentAprilTagPos.getY() - follower.getPose().getY()),(currentAprilTagPos.getX() - follower.getPose().getX()));



            telemetry.addData("Turning angle" ,turnAngle(follower.getPose().getHeading(),angleToAprilTag));
            telemetry.addData("Angle to aprilTag" , angleToAprilTag);


            if(Math.abs(turnAngle(follower.getPose().getHeading(),angleToAprilTag)) > Math.PI/6) {




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
                follower.setPose(new Pose(initPose.getX(), initPose.getY(), follower.getPose().getHeading()));
                state = "looking_for_april_tag";
                

                fl.setPower(0);
                fr.setPower(0);
                br.setPower(0);
                bl.setPower(0);


                initpos = turnAngle(follower.getPose().getHeading(),angleToAprilTag) * (0.45)/(Math.PI);


                resetElapsedtime.reset();


            }



        }




        if(resetElapsedtime.seconds()> 4){
            resetting = false;
        }






        if(state.equals("looking_for_april_tag") && !resetting && !moving) {




            if (elapsedTime.seconds() <= ShootConstants.aprilTagMoveScanTimePerStep_seconds) {
                waiting = false;
            } else if (elapsedTime.seconds() >= ShootConstants.aprilTagMoveScanTimePerStep_seconds && elapsedTime.seconds() <= ShootConstants.aprilTagMoveScanTimePerStep_seconds + ShootConstants.aprilTagScanTimePerStep_seconds) {
                waiting = true;
            } else if (elapsedTime.seconds() > ShootConstants.aprilTagMoveScanTimePerStep_seconds + ShootConstants.aprilTagScanTimePerStep_seconds) {
                waiting = false;
                elapsedTime.reset();

                initpos = pan.getPosition();
            }
                boolean found = false;

                //telemetry.addData("Num : ", aprilTagWebCam.getDetectedTags().size());



                for(int i =0; i < aprilTagWebCam.getDetectedTags().size();i++){
                    if(aprilTagWebCam.getDetectedTags().get(i).id == currentID){



                        state = "found_tag_orienting";
                        aprilTag = aprilTagWebCam.getDetectedTags().get(i);
                        found = true;

                        numshots++;
                        pan.setPosition(pan.getPosition());

                    }
                }

                if(!found){
                    if (!waiting){
                        if(initpos > 0.45){


                            resetElapsedtime.reset();
                            resetting = true;
                            initpos = 0;
                            resetElapsedtime.reset();
                        }
                        pan.setPosition(initpos + 0.025f);
                    }
                }



        }

        if(state.equals("found_tag_orienting") && !moving){

            double distance = aprilTag.ftcPose.y;

            if(!isAutoShot) {
                flywheelPower = ShootConstants.powerFromDistance(distance);

                tilt.setPosition(ShootConstants.tiltFromDistance(distance));
            }


            positionnecessary = pan.getPosition() +aprilTag.ftcPose.bearing * 0.45/180 + aprilTag.ftcPose.yaw * 5/45 *0.45/180;



            initposforseeingpreorient = (float)pan.getPosition();

            if((aprilTag.ftcPose.bearing * 0.45/180 + aprilTag.ftcPose.yaw * 5/45 *0.45/180)/0.45*180 > 3)pan.setPosition(positionnecessary);

            //telemetry.addData("position going to" ,  pan.getPosition() + aprilTag.ftcPose.bearing * 0.4/180);

            //tilt.setPosition(tilt.getPosition() + currOne.ftcPose.elevation);


            state = "shoot";
        }



        
        if(state.equals("shoot")){

            if(waitrotate.seconds() < 5 && !waitedforautorotate){
                return;
            }else{
                if(!waitedforautorotate)elapsedTime.reset();
                waitedforautorotate = true;
            }


            if(isAutoShot){
                state = "cleanup_shoot";

                flywheel.setPower(flywheelPower);


                elapsedTime.reset();
            }





            if(Math.abs(pan.getPosition() - positionnecessary) < 0.01){
                state = "cleanup_shoot";

                flywheel.setPower(flywheelPower);
                elapsedTime.reset();

            }

            aprilTagWebCam.displayDetectionTelemetry(aprilTag);

        }


        if(state.equals("cleanup_shoot")){

            if(elapsedTime.seconds() >= ShootConstants.flywheelAccelerationTime_seconds){
                transfer.setPower(1);
                if(!isAutoShot)aprilTagWebCam.displayDetectionTelemetry(aprilTag);
            }

            if(elapsedTime.seconds() >= ShootConstants.shotDuration_seconds + ShootConstants.flywheelAccelerationTime_seconds){
                shotbegan = false;
                isAutoShot = false;

                resetting = true;

                pan.setPosition(0);
                flywheel.setPower(0);
                transfer.setPower(-1);




                state = "";

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

    public static double distTo(Pose tagPos, Pose ourPos){
        return Math.sqrt((ourPos.getY() - tagPos.getY())*(ourPos.getY() - tagPos.getY()) + (ourPos.getX() - tagPos.getX())*(ourPos.getX() - tagPos.getX()));
    }

    public static double AngleToTag(Pose tagPos, Pose ourPos){
        return Math.atan2(tagPos.getY() - ourPos.getY(), tagPos.getX() - ourPos.getX());
    }


}


