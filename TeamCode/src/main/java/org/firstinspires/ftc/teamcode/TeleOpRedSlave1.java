package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.mechanisms.HoodedShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp



public class TeleOpRedSlave1 extends OpMode {
    private DcMotor pan;
    boolean beganshot = false;

    double initpos = 0f;

    private DcMotor fr;

    private DcMotor fl;

    private DcMotor br;
    private DcMotor bl;




    FieldRelativeDrive fieldDrive;
    ElapsedTime elapsedTime;

    private DcMotor intake;





    int state = 0;

    //0 : looking for
    //1 : orienting correctly


    ElapsedTime timer;



    double positionnecessary = 0f;




    Pose aprilTagRed = new Pose(144,144);
    Pose aprilTagBlue = new Pose(0,144);

    HoodedShooter hShooter;

    ElapsedTime rewindTimer;

    GoBildaPinpointDriver pinpoint;


    Servo arm1;

    Limelight3A limelight;
    boolean rewinding = false;
    int num = 0;
    public void init() {

        rewindTimer = new ElapsedTime();
        rewindTimer.reset();

        num = 0;

//        follower = Constants.createFollower(hardwareMap);
//
//        follower.setPose(new Pose(0,144));


        fieldDrive = new FieldRelativeDrive();

        fieldDrive.init(hardwareMap);







        timer = new ElapsedTime();
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        pan = hardwareMap.get(DcMotor.class, "rot");


        arm1 = hardwareMap.get(Servo.class, "arm1");
        intake = hardwareMap.get(DcMotor.class, "intake");


        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");


        pan = hardwareMap.get(DcMotor.class,"rot");
        pan.setTargetPosition(0);
        pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,96, 24, AngleUnit.RADIANS,0));



        intake = hardwareMap.get(DcMotor.class, "intake");

        //intake.setDirection(DcMotor.Direction.REVERSE);


        timer.reset();
        beganshot = false;
        elapsedTime = new ElapsedTime();

        elapsedTime.reset();

        hShooter = new HoodedShooter();




        hShooter.init(hardwareMap,telemetry, pinpoint);

        state=  0;
        intake.setPower(0);

    }

    @Override
    public void start(){
        intake.setPower(1);


    }
    double val = 0.6;

    public void loop(){


        if(gamepad2.b){
            //rewindTimer.reset();
            if(intake.getPower() == 1 )intake.setPower(-1);


            //rewinding = true;
        }else{

            if(intake.getPower() == -1)intake.setPower(1);
        }






        fieldDrive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0)fieldDrive.driveFieldRelative(-gamepad2.left_stick_y/4, gamepad2.left_stick_x/4, gamepad2.right_stick_x/4);


        if(gamepad2.leftBumperWasPressed()){
            intake.setPower(0.25);
        }





        if(gamepad2.rightBumperWasPressed()){
            intake.setPower(1);
        }

        if(gamepad1.square){

            telemetry.addData("We are on square", "hooray");
            hShooter.Fire(1);
        }
        if(gamepad1.circleWasPressed()){

            telemetry.addData("We are on circle", "Wow");
            hShooter.Fire(2);
        }

        if(gamepad1.triangleWasPressed()){

            telemetry.addData("We are on triangle", "Wow");
            hShooter.Fire(3);
        }


        hShooter.loop();


        //telemetry.update();
    }


}
