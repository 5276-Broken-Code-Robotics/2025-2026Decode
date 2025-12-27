package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.POVDrive;

@TeleOp(name = "EthanTeleopTest")

public class EthanTeleopTest extends OpMode {

    POVDrive drive = new POVDrive();
    DcMotor intake;
    DcMotor flywheel;

    boolean startRunningTransfer;

    ElapsedTime timer;

    CRServo transfer;

    Servo tilt;

    @Override
    public void init() {
        drive.init(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        tilt = hardwareMap.get(Servo.class, "tilt");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        intake.setDirection(DcMotor.Direction.REVERSE);
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        intake.setPower(1);
        tilt.setPosition(0.59 * gamepad1.left_trigger);

        flywheel.setPower(gamepad1.right_trigger);

        if(gamepad1.triangle) {
            timer.reset();
            startRunningTransfer = true;
        }

        if(startRunningTransfer && timer.seconds() <= 1){
            transfer.setPower(1);
        } else {
            transfer.setPower(-1);
            startRunningTransfer = false;
        }
    }
}
