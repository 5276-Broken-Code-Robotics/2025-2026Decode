package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.doclint.Messages;

@TeleOp(group = "Tests")
public class ServoStep extends OpMode {

    Servo servo;

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};
    int stepIndex = 0;

    @Override
    public void init() {

        servo = hardwareMap.get(Servo.class, "arm2");
        servo.setPosition(0);

    }

    @Override
    public void loop() {

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadUpWasPressed()) {servo.setPosition(servo.getPosition() + stepSizes[stepIndex]);}
        if (gamepad1.dpadDownWasPressed()) {servo.setPosition(servo.getPosition() - stepSizes[stepIndex]);}

        telemetry.addData("Servo Pos", servo.getPosition());
        telemetry.addData("Step Size", "%.6f (B Button)", stepSizes[stepIndex]);

    }
}
