
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


public class orientAndLocalTest extends OpMode {



    private Servo pan;



    //0.75 for close sec 2


    public boolean isRed = false;


    CRServo transfer;

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


    FieldRelativeDrive fieldDrive;


    private DcMotor fr;

    private DcMotor fl;


    private DcMotor br;
    private DcMotor bl;



    ElapsedTime resetTimer = new ElapsedTime();


    // (160 (vbig gear) / 18 (small gear)  * 20 (degrees of rotation))/300
    double maxTilt = 0.59;
    public void init() {


        resetTimer.reset();





//        follower = Constants.createFollower(hardwareMap);
//
//        follower.setPose(new Pose(0,144));


        fieldDrive = new FieldRelativeDrive();

        //fieldDrive.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);




        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));


        follower.setPose(new Pose(0, 0, Math.toRadians(0)));

        follower.update();

        pan = hardwareMap.get(Servo.class, "pan");



        currentAprilTagPos = aprilTagRed;


        pan.setPosition(0);



    }

    public void start(){


    }




    double binit = 0f;
    public void loop()
    {

        fieldDrive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        if(gamepad1.bWasPressed()){
            binit = pan.getPosition()+0.1f;
        }


        pan.setPosition(binit);

        //Orienting();

        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Heading", follower.getHeading());



        telemetry.addData("Follower X : ", follower.getPose().getX());

        telemetry.addData("Follower Y :" , follower.getPose().getY());

        telemetry.update();

        follower.update();
    }


    public void Orienting(){


        angleToAprilTag = Math.atan2((currentAprilTagPos.getY() - follower.getPose().getY()),(currentAprilTagPos.getX() - follower.getPose().getX()));
        double initpos = 0;
        initpos = turnAngle(follower.getPose().getHeading(),angleToAprilTag) * (0.45)/(Math.PI);
        pan.setPosition(initpos);


        telemetry.addData("Pos calced : ", initpos);
        telemetry.addData("Pan position : ", pan.getPosition());





        /*
        for(int i =0; i < aprilTagWebCam.getDetectedTags().size();i++){
            if(aprilTagWebCam.getDetectedTags().get(i).id == currentID){
                aprilTag = aprilTagWebCam.getDetectedTags().get(i);
                Pose realPose = new Pose(currentAprilTagPos.getX() -( aprilTag.ftcPose.x * Math.cos(follower.getHeading()) -aprilTag.ftcPose.y * Math.sin(follower.getHeading())),currentAprilTagPos.getY() - ( aprilTag.ftcPose.x * Math.sin(follower.getHeading())  + aprilTag.ftcPose.y * Math.cos(follower.getHeading())));
                follower.setPose(realPose);
            }
        }

         */

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


