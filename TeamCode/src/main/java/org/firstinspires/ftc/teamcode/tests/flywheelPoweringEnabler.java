package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class flywheelPoweringEnabler extends OpMode {


    DcMotor f1;
    DcMotor f2;
    public void init(){
        f1 = hardwareMap.get(DcMotor.class, "flywheel1");

        f1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void loop(){
        f1.setPower(0.5);
        telemetry.addData("Hello", "World" );
    }

    public void start(){


    }
}
