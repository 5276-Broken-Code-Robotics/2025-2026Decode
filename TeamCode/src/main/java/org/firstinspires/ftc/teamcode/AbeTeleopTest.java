package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class AbeTeleopTest extends OpMode {
    DcMotor intake;
    DcMotor flywheel;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

    }
    @Override
    public void loop(){
        intake.setPower(gamepad1.left_stick_y);
    }
}
