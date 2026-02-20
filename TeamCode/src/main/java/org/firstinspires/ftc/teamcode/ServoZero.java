package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoZero")
public class ServoZero extends OpMode {

    Servo tilt;

    @Override
    public void init() {

        tilt = hardwareMap.get(Servo.class, "tilt");
        tilt.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {

        tilt.setPosition(0);

    }
}
