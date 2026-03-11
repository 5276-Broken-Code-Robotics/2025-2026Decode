package org.firstinspires.ftc.teamcode.tests;

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
@TeleOp(group = "Tests")



public class hoodTester extends OpMode {



    private Servo tilt;



    double pos = 0;

    public void init() {
        pos = 0.3;
        tilt = hardwareMap.get(Servo.class, "tilt");

    }








    public void loop() {
        if(gamepad1.leftBumperWasPressed()){
            pos += 0.1;
        }
        if(gamepad1.rightBumperWasPressed()){
            pos -= 0.1;
        }

        tilt.setPosition(pos);


        telemetry.addData("Position :", pos);

        telemetry.update();


    }

}
