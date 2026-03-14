package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FinalFreeSortHSV;

@TeleOp(group = "Tests")
public class FreeSortTest extends OpMode {

    FinalFreeSortHSV freesort = new FinalFreeSortHSV();


    public void init(){

        freesort.init(hardwareMap, telemetry);

    }

    public void loop(){
        freesort.loop();

        //if (gamepad1.right_trigger > .25) freesort.shootAll();
        //if (gamepad1.right_bumper) freesort.shootPurple();
        //if (gamepad1.left_bumper) freesort.shootGreen();

        if (gamepad1.dpadDownWasPressed()) {freesort.shoot(1);}
        if (gamepad1.dpadRightWasPressed()) {freesort.shoot(2);}
        if (gamepad1.dpadLeftWasPressed()) {freesort.shoot(3);}

        freesort.loop();

    }


}
