package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends OpMode {

    DcMotor rot;

    public void init(){

        rot = hardwareMap.get(DcMotor.class, "rot");
        rot.setTargetPosition(384);
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setPower(1);
    }

    public void loop(){

    }

}
