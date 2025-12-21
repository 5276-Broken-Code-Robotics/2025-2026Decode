package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeTesting")

public class IntakeTest extends OpMode {

    DcMotor intake;
    DcMotor intake1;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.left_stick_y);
    }
}
