package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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
