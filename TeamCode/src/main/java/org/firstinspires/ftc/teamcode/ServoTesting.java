package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "ServoTest", group = "testing")
public class ServoTesting extends OpMode {

    ServoEx servo;

    public void init() {

        servo = new SimpleServo(hardwareMap, "servo", 0, 360);

    }


    public void loop() {
        if (gamepad1.a) servo.rotateByAngle(90);

    }

}
