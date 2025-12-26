package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "FlywheelTest")
public class FlywheelTest extends OpMode {

    DcMotor flywheel;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        flywheel.setPower(gamepad2.right_trigger);
    }
}
