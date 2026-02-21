package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends OpMode {

    DcMotor rot;

    int ticknum = 0;

    public void init(){



        rot = hardwareMap.get(DcMotor.class, "rot");
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rot.setTargetPosition(0);
        rot.setPower(1);



    }

    public void loop(){


        if(gamepad1.xWasPressed()){
            ticknum = 0;
        }
        if(gamepad1.squareWasPressed()){
            ticknum += 20;
        }

        if(gamepad1.triangleWasPressed()){
            ticknum -= 20 ;
        }

        if(ticknum > 143){
            ticknum = 143;
        }

        if(ticknum < - 143){
            ticknum = 143;
        }
        rot.setTargetPosition(ticknum);


        telemetry.addData("Tick num", ticknum);
    }

}
