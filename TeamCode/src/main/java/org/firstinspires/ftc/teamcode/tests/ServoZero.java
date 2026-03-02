package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Tests")
public class ServoZero extends OpMode {

    Servo servo;

    @Override
    public void init() {

        servo = hardwareMap.get(Servo.class, "arm2");
        servo.setPosition(.4);
    }

    @Override
    public void loop() {

        servo.setPosition(.4);

    }
}
