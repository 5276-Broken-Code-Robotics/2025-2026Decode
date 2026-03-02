package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Tests")
public class ServoPorportional extends OpMode {

    Servo arm1;
    Servo arm2;
    Servo arm3;

    DcMotor flywheel1;
    DcMotor flywheel2;

    @Override
    public void init() {

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 =  hardwareMap.get(Servo.class, "arm2");
        arm3 =  hardwareMap.get(Servo.class, "arm3");

        arm1.scaleRange(0.03, .45);
        arm2.scaleRange(0.03, .45);
        arm3.scaleRange(0.03, .45);

        arm1.setPosition(0);
        arm2.setPosition(0);
        arm3.setPosition(0);

        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

        flywheel1.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        arm1.setPosition(gamepad1.right_trigger);
        arm2.setPosition(gamepad1.left_trigger);
        arm3.setPosition(gamepad1.right_stick_x);

        telemetry.addData("Arm1", arm1.getPosition());
        telemetry.addData("Arm2", arm2.getPosition());
        telemetry.addData("Arm3", arm3.getPosition());

        flywheel1.setPower(gamepad1.left_stick_y);
        flywheel2.setPower(gamepad1.left_stick_y);



    }
}
