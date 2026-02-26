package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "FlywheelTest")
public class FlywheelTest extends OpMode {

    DcMotor flywheel1;
    DcMotor flywheel2;

    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        flywheel1.setPower(gamepad1.right_stick_y);
        flywheel2.setPower(gamepad1.right_stick_y);
    }
}
