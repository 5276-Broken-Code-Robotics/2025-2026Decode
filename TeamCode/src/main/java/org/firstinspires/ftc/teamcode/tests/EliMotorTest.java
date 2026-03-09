package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Tests")
public class EliMotorTest extends OpMode {

    DcMotor rot;


    @Override
    public void init(){

        rot = hardwareMap.get(DcMotor.class, "rot");


    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            rot.setTargetPosition((int)(rot.getCurrentPosition()+384.5));
            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rot.setPower(1);
        }

        telemetry.addData("Position", rot.getCurrentPosition());

    }
}
