package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class onlyPan extends OpMode {
    DcMotor pan;
    public void init(){
        pan = hardwareMap.get(DcMotorEx.class, "rot");
    }


    int count = 0;

    int pos = 0;
    public void loop(){
        if(gamepad1.rightBumperWasPressed()){
            pos+=15;
            count++;
        }

        if(gamepad1.leftBumperWasPressed()){
            pos-=15;
            count++;
        }

        pan.setTargetPosition(pos);



        telemetry.addData("Current position on pan", pan.getCurrentPosition());
        telemetry.addData("Target position on pan", pan.getTargetPosition());

        telemetry.addData("Pos", pos);
        telemetry.update();
    }

    public void start(){

    }

}
