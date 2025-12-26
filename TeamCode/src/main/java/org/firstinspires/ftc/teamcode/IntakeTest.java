package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.POVDrive;

@TeleOp(name = "IntakeTesting")

public class IntakeTest extends OpMode {

    POVDrive drive = new POVDrive();
    DcMotor intake;


    @Override
    public void init() {

        drive.init(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        intake.setPower(1);


    }
}
