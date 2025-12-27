package org.firstinspires.ftc.teamcode;

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
    CRServo transfer;
    Servo tilt;
    Servo pan;

    ElapsedTime timer;

    boolean isShooting;
    boolean enableIntake;

    @Override
    public void init() {
        drive.init(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        tilt = hardwareMap.get(Servo.class, "tilt");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        pan = hardwareMap.get(Servo.class, "pan");

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transfer.setPower(-1);
        flywheel.setPower(0.7);
    }

    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        tilt.setPosition(0.59 * gamepad1.left_trigger);

        managePan();
        manageIntake();

        if(gamepad1.right_trigger >= 0.1 && !isShooting) {
            shoot();
        } else if (isShooting) {
            manageShoot();
        }
    }

    private void managePan() {
        double increment = 0.01;

        if(gamepad1.dpad_left) {
            pan.setPosition(Math.min(pan.getPosition() + increment, 1));
        } else if(gamepad1.dpad_right) {
            pan.setPosition(Math.max(pan.getPosition() - increment, 0));
        }
    }

    private void manageIntake() {
        if(gamepad1.circleWasPressed()) {
            enableIntake = !enableIntake;
        }

        if(enableIntake) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    private void shoot() {
        isShooting = true;

        transfer.setPower(1);
        timer.reset();
    }

    private void manageShoot() {
        if(timer.seconds() >= 1) {
            isShooting = false;

            transfer.setPower(-1);
        }
    }
}