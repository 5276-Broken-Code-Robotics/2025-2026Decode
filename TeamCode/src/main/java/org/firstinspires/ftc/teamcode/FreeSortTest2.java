package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.FreeSortHSV;

@TeleOp
public class FreeSortTest2 extends OpMode {

    Servo arm1;
    Servo arm2;
    Servo arm3;



    @Override
    public void init() {

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");

        

    }

    @Override
    public void loop() {

        arm2.setPosition((.4 * (double) gamepad1.right_trigger));
        arm3.setPosition((.4 * (double) gamepad1.left_trigger));

        /*
        arm1.setPosition(.15 * gamepad2.right_trigger);
        arm2.setPosition((.15 * gamepad2.right_trigger));
        arm3.setPosition((.3 * gamepad2.right_trigger));
        */

    }

}
