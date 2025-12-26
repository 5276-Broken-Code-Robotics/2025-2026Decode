package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoZero")
public class ServoZero extends OpMode {

    Servo pan;

    @Override
    public void init() {

        pan = hardwareMap.get(Servo.class, "pan");
        
    }

    @Override
    public void loop() {

        pan.setPosition(0);

    }
}
