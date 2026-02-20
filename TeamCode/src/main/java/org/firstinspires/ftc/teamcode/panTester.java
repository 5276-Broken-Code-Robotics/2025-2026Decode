package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class panTester extends OpMode {


    Follower follower;


    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor pan;
    int jklm = 0;
    private GoBildaPinpointDriver pinpoint;
    Limelight3A limelight3A;
    @Override
    public void init(){


        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");


        pan = hardwareMap.get(DcMotor.class,"rot");
        pan.setTargetPosition(0);
        pan.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();




        follower = Constants.createFollower(hardwareMap);

        follower.setPose(new Pose(0,0,0));
        pan.setPower(1);


        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);


    }
    @Override
    public void start(){
        limelight3A.start();

    }
    @Override
    public void loop(){
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);



        follower.update();

        telemetry.addData("Runnning?" , limelight3A.isRunning());
        pinpoint.update();
        double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        limelight3A.updateRobotOrientation(robotHeading + pan.getCurrentPosition() / 537.7/2 * PI);


        double angleToAprilTag = Math.atan2((144 - pinpoint.getPosY(DistanceUnit.INCH)),(144 - pinpoint.getPosX(DistanceUnit.INCH)));

        float initpos = (float) (turnAngle(pinpoint.getHeading(AngleUnit.RADIANS),angleToAprilTag) * (537.7/2)/(Math.PI));





        telemetry.addData("Init pos variable : ", initpos);


        telemetry.addData("Turn angle : ", turnAngle(pinpoint.getHeading(AngleUnit.RADIANS),angleToAprilTag) * 180/PI);

        telemetry.addData("Angle to tag : ", angleToAprilTag * 180/PI);

        if(gamepad1.aWasPressed())pan.setTargetPosition((int)initpos);

        if(gamepad1.rightBumperWasPressed()){
            pan.setTargetPosition(pan.getTargetPosition()-15);
        }

        if(gamepad1.leftBumperWasPressed()){
            pan.setTargetPosition(pan.getTargetPosition()+15);
        }

        telemetry.addData("Current position on pan", pan.getCurrentPosition());


        telemetry.update();

    }

    public static double turnAngle(double currentHeading, double targetAngle) {
        double error = targetAngle - currentHeading;

        // Normalize to (-PI, PI]
        while (error > Math.PI)  error -= 2 * Math.PI;
        while (error <= -Math.PI) error += 2 * Math.PI;

        return error;
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
